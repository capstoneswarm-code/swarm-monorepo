"""Create a cooperative runtime COPY; never write to wheel_ekf_demo."""
import ast
from pathlib import Path
import shutil

import yaml


def statements(code):
    return ast.parse(code).body


class ExplorerPatch(ast.NodeTransformer):
    def __init__(self):
        self.counts = dict(init=0, map=0, step=0, finish=0, choose=0, send=0)
        self.in_explorer = False

    def visit_ClassDef(self, node):
        if node.name != 'Explorer':
            return self.generic_visit(node)
        self.in_explorer = True
        node = self.generic_visit(node)
        self.in_explorer = False
        return node

    def visit_FunctionDef(self, node):
        if not self.in_explorer:
            return self.generic_visit(node)
        node = self.generic_visit(node)
        if node.name == '__init__':
            # Place after ordinary state initialization, before the step timer.
            i = next((i for i, item in enumerate(node.body)
                if isinstance(item, ast.Assign) and any(
                    isinstance(t, ast.Attribute) and t.attr == 'timer' for t in item.targets)), None)
            if i is None:
                raise ValueError('Explorer.__init__ has no recognized timer assignment')
            node.body[i:i] = statements('self.cooperation = Cooperation(self)')
            self.counts['init'] += 1
            for call in ast.walk(node):
                if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                        and call.func.attr == 'create_subscription' and len(call.args) >= 3
                        and isinstance(call.args[2], ast.Attribute) and call.args[2].attr == 'on_map'):
                    call.args[1] = ast.Constant('/swarm/merged_map')
                    self.counts['map'] += 1
        elif node.name == 'step':
            node.body[0:0] = statements('self.cooperation.before_step(self)')
            self.counts['step'] += 1
        elif node.name == 'finish':
            node.body[0:0] = statements('self.cooperation.release()')
            self.counts['finish'] += 1
        elif node.name == 'no_frontier':
            node.body[0:0] = statements('''
if self.cooperation.peer_exploring():
    self.no_frontier_since = None
    self.note("Waiting: peer is still exploring")
    return
''')
        return node

    def visit_Assign(self, node):
        if (self.in_explorer and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Name) and node.value.func.id == 'choose_goal'):
            first = node.value.args[0]
            if not isinstance(first, ast.Name):
                raise ValueError('Expected a named candidate mask in choose_goal')
            name = first.id
            self.counts['choose'] += 1
            return statements(f'''
{name}, peer_reserved_all = self.cooperation.filter({name}, self.geometry)
if peer_reserved_all:
    self.no_frontier_since = None
    self.note("Waiting: peer reserved the remaining frontiers")
    return
''') + [self.generic_visit(node)]
        return self.generic_visit(node)

    def visit_Call(self, node):
        if (self.in_explorer and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'send_goal_async'
                and isinstance(node.func.value, ast.Attribute)
                and node.func.value.attr == 'navigate' and len(node.args) == 1):
            self.counts['send'] += 1
            return ast.Call(func=ast.Attribute(value=ast.Attribute(value=ast.Name(id='self', ctx=ast.Load()),
                attr='cooperation', ctx=ast.Load()), attr='send_goal', ctx=ast.Load()),
                args=[node.func.value, node.args[0], ast.Name(id='self', ctx=ast.Load())], keywords=node.keywords)
        return self.generic_visit(node)


def patch_explorer(source):
    tree = ast.parse(source)
    if any(isinstance(n, ast.Attribute) and n.attr == 'cooperation' for n in ast.walk(tree)):
        raise ValueError('Source explorer is already patched; use the unmodified single-robot source')
    patch = ExplorerPatch()
    tree = patch.visit(tree)
    for name in ('init', 'map', 'step', 'finish', 'choose'):
        if patch.counts[name] != 1:
            raise ValueError(f'Explorer layout changed: expected one {name}, found {patch.counts[name]}')
    if patch.counts['send'] < 1:
        raise ValueError('Explorer has no recognized Nav2 goal submission')
    # Keep module docstring and __future__ imports in their required positions.
    i = 0
    while i < len(tree.body) and (isinstance(tree.body[i], ast.Expr)
            and isinstance(tree.body[i].value, ast.Constant) and isinstance(tree.body[i].value.value, str)
            or isinstance(tree.body[i], ast.ImportFrom) and tree.body[i].module == '__future__'):
        i += 1
    tree.body[i:i] = statements('from cooperative_hooks import Cooperation')
    ast.fix_missing_locations(tree)
    result = ast.unparse(tree)+'\n'
    compile(result, '<cooperative explorer>', 'exec')
    return result


def prepare(root, robot):
    root = Path(root).resolve()
    source = root/'src/wheel_ekf_demo'
    folder = root/'src/cooperative_mapping/runtime'/robot
    required = ['nav2.launch.py', 'mapping.launch.py', 'frontier_explorer.py',
                'robot_demo.py', 'hardware.py', 'demo_core.py', 'ekf.yaml',
                f'nav2_params_{robot}.yaml', 'manual_nav.xml', 'exploration_nav.xml']
    for name in required:
        if not (source/name).is_file():
            raise FileNotFoundError(f'Missing current single-robot file: {source/name}')
    patched = patch_explorer((source/'frontier_explorer.py').read_text())
    launch_source = (source/'nav2.launch.py').read_text()
    launch_tree = ast.parse(launch_source)
    roots = [n for n in launch_tree.body if isinstance(n, ast.Assign)
             and any(isinstance(t, ast.Name) and t.id == 'FOLDER' for t in n.targets)]
    if len(roots) != 1 or '__file__' not in ast.unparse(roots[0].value):
        raise ValueError('Expected launch FOLDER to derive from __file__; refusing to risk writing outside the runtime copy')
    # All validation before creating/updating the cooperative runtime.
    config_path = source/f'nav2_params_{robot}.yaml'
    config = yaml.safe_load(config_path.read_text())
    config['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'] = '/swarm/merged_map'
    # Normalize ONLY behavior-tree path settings, without touching calibration.
    nav = config['bt_navigator']['ros__parameters']
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        if nav.get(key):
            name = Path(nav[key]).name
            if not (source/name).is_file():
                raise FileNotFoundError(f'Missing behavior tree: {source/name}')
            nav[key] = str(folder/name)
    folder.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        if path.is_file() and path.suffix in ('.py', '.yaml', '.xml', '.sh') and 'generated' not in path.name:
            shutil.copy2(path, folder/path.name)
    (folder/'frontier_explorer.py').write_text(patched)
    (folder/config_path.name).write_text(yaml.safe_dump(config, sort_keys=False))
    # Compatibility with launch versions that still load the unsuffixed name.
    (folder/'nav2_params.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    # Some launch files use a different parameter source; refuse a mismatch.
    launch = (folder/'nav2.launch.py').read_text()
    if 'nav2_params' not in launch:
        raise ValueError('Launch file does not select a recognized Nav2 parameter file')
    viewer = folder/'web_map.py'
    if viewer.is_file():
        class ViewerMap(ast.NodeTransformer):
            def visit_Constant(self, node):
                if node.value == '/map':
                    return ast.copy_location(ast.Constant('/swarm/merged_map'), node)
                return node
        tree = ViewerMap().visit(ast.parse(viewer.read_text()))
        ast.fix_missing_locations(tree)
        viewer.write_text(ast.unparse(tree)+'\n')
    return folder
