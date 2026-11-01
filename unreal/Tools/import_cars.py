# Imports modular car models into /Game/TT/Cars and writes the catalog the game reads
# (Content/TT/Cars/cars.json). Run from import_cars.bat (editor closed).
#
# Source layout (one folder per car, like most modular kits):
#   <CarModels>/<Car name>/<Category>/<Variant>/<files .fbx/.obj/.gltf/.glb + textures>
#   e.g. CarModels/My Coupe/Front Bumper/BumperFront_stock/BumperFront_stock.fbx
# Variants that share the text before the first "_" are alternatives of one part (BumperFront_stock,
# BumperFront_Sport ...); different prefixes in one category are separate parts (SeatLeft, SeatRight).
# All parts of a car must be modelled in the same car space (that is how modular kits are made).
#
# Source folder: environment variable TT_CARS, else <project>/../CarModels.
# Already imported variants are skipped (set TT_REIMPORT=1 to import again).
import json
import os
import re

import unreal

AT = unreal.AssetToolsHelpers.get_asset_tools()
EAL = unreal.EditorAssetLibrary
EXT = ('.fbx', '.obj', '.gltf', '.glb')
DEST = '/Game/TT/Cars'

project = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
src = os.environ.get('TT_CARS') or os.path.normpath(os.path.join(project, '..', 'CarModels'))
reimport = os.environ.get('TT_REIMPORT') == '1'


def safe(name):
    s = re.sub(r'[^A-Za-z0-9_]+', '_', name).strip('_')
    return s or 'X'


def static_meshes(path):
    out = []
    for a in EAL.list_assets(path, recursive=True, include_folder=False):
        data = EAL.find_asset_data(a)
        if str(data.asset_class_path.asset_name) == 'StaticMesh':
            out.append(str(data.package_name) + '.' + str(data.asset_name))
    return sorted(out)


def import_variant(files, dest):
    if EAL.does_directory_exist(dest) and static_meshes(dest) and not reimport:
        return static_meshes(dest), False
    tasks = []
    for f in files:
        t = unreal.AssetImportTask()
        t.set_editor_property('filename', f)
        t.set_editor_property('destination_path', dest)
        t.set_editor_property('automated', True)
        t.set_editor_property('replace_existing', True)
        t.set_editor_property('save', True)
        tasks.append(t)
    AT.import_asset_tasks(tasks)
    meshes = static_meshes(dest)
    # the game deforms the meshes on the CPU: keep their vertex data in memory
    for m in meshes:
        sm = unreal.load_asset(m)
        if sm and not sm.get_editor_property('allow_cpu_access'):
            sm.set_editor_property('allow_cpu_access', True)
            EAL.save_loaded_asset(sm)
    return meshes, True


cars = []
imported = 0
if not os.path.isdir(src):
    unreal.log_error('TT import_cars: no folder %s (set TT_CARS)' % src)
else:
    for car in sorted(os.listdir(src)):
        car_dir = os.path.join(src, car)
        if not os.path.isdir(car_dir):
            continue
        parts = {}
        for cat in sorted(os.listdir(car_dir)):
            cat_dir = os.path.join(car_dir, cat)
            if not os.path.isdir(cat_dir):
                continue
            # a category may hold model files directly (one variant) or variant folders
            entries = [(cat, cat_dir)] if any(f.lower().endswith(EXT) for f in os.listdir(cat_dir)) else \
                [(v, os.path.join(cat_dir, v)) for v in sorted(os.listdir(cat_dir)) if os.path.isdir(os.path.join(cat_dir, v))]
            for var, var_dir in entries:
                files = [os.path.join(var_dir, f) for f in sorted(os.listdir(var_dir)) if f.lower().endswith(EXT)]
                if not files:
                    continue
                dest = '%s/%s/%s/%s' % (DEST, safe(car), safe(cat), safe(var))
                try:
                    meshes, new = import_variant(files, dest)
                except Exception as e:
                    unreal.log_error('TT import_cars: %s/%s/%s failed: %s' % (car, cat, var, e))
                    continue
                imported += 1 if new else 0
                if not meshes:
                    unreal.log_warning('TT import_cars: %s/%s/%s: no static meshes' % (car, cat, var))
                    continue
                group = var.split('_')[0] if '_' in var else var
                key = cat + '/' + group
                parts.setdefault(key, {'category': cat, 'group': group, 'variants': {}})
                parts[key]['variants'][var] = meshes
        if parts:
            cars.append({'name': car, 'parts': list(parts.values())})
            unreal.log('TT import_cars: %s: %d parts' % (car, len(parts)))

out_dir = os.path.join(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_content_dir()), 'TT', 'Cars')
os.makedirs(out_dir, exist_ok=True)
with open(os.path.join(out_dir, 'cars.json'), 'w', encoding='utf-8') as f:
    json.dump({'cars': cars}, f, ensure_ascii=False, indent=1)
print('TT_CARS_DONE cars=%d imported_variants=%d source=%s' % (len(cars), imported, src))
