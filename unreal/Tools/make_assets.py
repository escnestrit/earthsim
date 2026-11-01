# Creates the game's materials in /Game/TT/Materials (run once, from make_assets.bat or
# Tools > Execute Python Script in the editor). The C++ code loads them by name and falls back to the
# engine's basic material when they are missing, so the game also runs without them.
#
# Every material has a "Color" vector parameter (the code tints with it).
# Car materials read vertex colour: R = paint scraped off / glass cracked, G = metal creased.
import unreal

AT = unreal.AssetToolsHelpers.get_asset_tools()
MEL = unreal.MaterialEditingLibrary
EAL = unreal.EditorAssetLibrary
ROOT = '/Game/TT/Materials'
MP = unreal.MaterialProperty
made, failed = [], []


def new_mat(name):
    path = ROOT + '/' + name
    if EAL.does_asset_exist(path):
        EAL.delete_asset(path)
    return AT.create_asset(name, ROOT, unreal.Material, unreal.MaterialFactoryNew())


def node(m, cls, x, y, **props):
    e = MEL.create_material_expression(m, cls, x, y)
    for k, v in props.items():
        e.set_editor_property(k, v)
    return e


def vec(m, name, rgb, x, y):
    return node(m, unreal.MaterialExpressionVectorParameter, x, y, parameter_name=name,
                default_value=unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))


def scal(m, name, v, x, y):
    return node(m, unreal.MaterialExpressionScalarParameter, x, y, parameter_name=name, default_value=v)


def const(m, v, x, y):
    return node(m, unreal.MaterialExpressionConstant, x, y, r=v)


def const3(m, rgb, x, y):
    return node(m, unreal.MaterialExpressionConstant3Vector, x, y, constant=unreal.LinearColor(rgb[0], rgb[1], rgb[2], 1.0))


def link(a, b, inp='', out=''):
    if not MEL.connect_material_expressions(a, out, b, inp):
        raise RuntimeError('cannot connect %s.%s -> %s.%s' % (a.get_name(), out, b.get_name(), inp))


def mul(m, a, b, x, y, aout='', bout=''):
    e = node(m, unreal.MaterialExpressionMultiply, x, y)
    link(a, e, 'A', aout)
    link(b, e, 'B', bout)
    return e


def mulc(m, a, k, x, y, aout=''):
    e = node(m, unreal.MaterialExpressionMultiply, x, y, const_b=k)
    link(a, e, 'A', aout)
    return e


def lerp(m, a, b, alpha, x, y, alpha_out=''):
    e = node(m, unreal.MaterialExpressionLinearInterpolate, x, y)
    link(a, e, 'A')
    link(b, e, 'B')
    link(alpha, e, 'Alpha', alpha_out)
    return e


def out(m, e, prop, o=''):
    if not MEL.connect_material_property(e, o, prop):
        raise RuntimeError('cannot connect %s to %s' % (e.get_name(), prop))


def world_uv(m, scale_cm, x, y):
    wp = node(m, unreal.MaterialExpressionWorldPosition, x, y)
    mask = node(m, unreal.MaterialExpressionComponentMask, x + 150, y, r=True, g=True, b=False, a=False)
    link(wp, mask)
    return mulc(m, mask, 1.0 / scale_cm, x + 300, y)


def tex(m, path, uv, x, y, normal=False):
    if not EAL.does_asset_exist(path):
        return None
    t = node(m, unreal.MaterialExpressionTextureSample, x, y, texture=unreal.load_asset(path))
    if normal:
        t.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL)
    link(uv, t, 'UVs')
    return t


def noise(m, scale, lo, hi, x, y, levels=4):
    return node(m, unreal.MaterialExpressionNoise, x, y, scale=scale, levels=levels, output_min=lo, output_max=hi)


def finish(m, name, two_sided=False):
    m.set_editor_property('two_sided', two_sided)
    m.set_editor_property('used_with_instanced_static_meshes', True)
    MEL.layout_material_expressions(m)
    MEL.recompile_material(m)
    EAL.save_asset(m.get_path_name())
    made.append(name)


def build(name, fn):
    try:
        fn(name)
    except Exception as e:  # keep going: one broken material must not stop the others
        failed.append('%s: %s' % (name, e))
        unreal.log_error('TT make_assets: %s failed: %s' % (name, e))


# ------------------------------------------------------------------ car

def car_paint(name):
    m = new_mat(name)
    m.set_editor_property('shading_model', unreal.MaterialShadingModel.MSM_CLEAR_COAT)
    col = vec(m, 'Color', (0.42, 0.03, 0.02), -900, 0)
    vc = node(m, unreal.MaterialExpressionVertexColor, -900, 300)
    # creases darken the paint (G), scraped areas show bare steel (R)
    crease = node(m, unreal.MaterialExpressionOneMinus, -700, 250)
    link(mulc(m, vc, 0.55, -800, 250, aout='G'), crease)
    painted = mul(m, col, crease, -550, 50)
    steel = const3(m, (0.56, 0.56, 0.58), -550, 200)
    out(m, lerp(m, painted, steel, vc, -350, 100, alpha_out='R'), MP.MP_BASE_COLOR)
    out(m, lerp(m, const(m, 0.45, -550, 350), const(m, 1.0, -550, 420), vc, -350, 350, alpha_out='R'), MP.MP_METALLIC)
    out(m, lerp(m, const(m, 0.32, -550, 500), const(m, 0.5, -550, 570), vc, -350, 500, alpha_out='R'), MP.MP_ROUGHNESS)
    out(m, lerp(m, const(m, 1.0, -550, 650), const(m, 0.0, -550, 720), vc, -350, 650, alpha_out='R'), MP.MP_CUSTOM_DATA0)  # clear coat
    out(m, const(m, 0.03, -350, 800), MP.MP_CUSTOM_DATA1)  # clear coat roughness
    finish(m, name, two_sided=True)


def glass(name):
    m = new_mat(name)
    col = vec(m, 'Color', (0.01, 0.012, 0.015), -700, 0)
    vc = node(m, unreal.MaterialExpressionVertexColor, -700, 250)
    cracked = const3(m, (0.55, 0.57, 0.58), -700, 120)
    out(m, lerp(m, col, cracked, vc, -400, 50, alpha_out='R'), MP.MP_BASE_COLOR)
    out(m, lerp(m, const(m, 0.03, -600, 400), const(m, 0.55, -600, 470), vc, -400, 400, alpha_out='R'), MP.MP_ROUGHNESS)
    out(m, const(m, 1.0, -400, 550), MP.MP_SPECULAR)
    finish(m, name, two_sided=True)


def trim(name):
    m = new_mat(name)
    out(m, vec(m, 'Color', (0.02, 0.02, 0.02), -500, 0), MP.MP_BASE_COLOR)
    out(m, const(m, 0.62, -500, 200), MP.MP_ROUGHNESS)
    finish(m, name, two_sided=True)


def light(name):
    m = new_mat(name)
    col = vec(m, 'Color', (1, 0.95, 0.85), -600, 0)
    out(m, mulc(m, col, 0.6, -350, 0), MP.MP_BASE_COLOR)
    out(m, mul(m, col, scal(m, 'Glow', 0.5, -600, 200), -350, 200), MP.MP_EMISSIVE_COLOR)
    out(m, const(m, 0.08, -350, 350), MP.MP_ROUGHNESS)
    finish(m, name, two_sided=True)


def tyre(name):
    m = new_mat(name)
    col = vec(m, 'Color', (0.025, 0.025, 0.025), -600, 0)
    out(m, col, MP.MP_BASE_COLOR)
    out(m, const(m, 0.86, -400, 200), MP.MP_ROUGHNESS)
    finish(m, name)


def rim(name):
    m = new_mat(name)
    out(m, vec(m, 'Color', (0.7, 0.7, 0.72), -500, 0), MP.MP_BASE_COLOR)
    out(m, const(m, 1.0, -500, 200), MP.MP_METALLIC)
    out(m, const(m, 0.22, -500, 300), MP.MP_ROUGHNESS)
    finish(m, name)


# ------------------------------------------------------------------ world (world-aligned, so scaled boxes need no UVs)

def asphalt(name):
    m = new_mat(name)
    col = vec(m, 'Color', (1, 1, 1), -1000, -200)
    macro = noise(m, 0.004, 0.75, 1.15, -1000, 0)  # patches and wear
    base = mul(m, const3(m, (0.055, 0.055, 0.058), -800, -100), macro, -600, -50)
    uv = world_uv(m, 140.0, -1400, 300)
    grain = tex(m, '/Game/StarterContent/Textures/T_Ground_Gravel_D', uv, -1000, 300)
    if grain:
        g = node(m, unreal.MaterialExpressionAdd, -800, 300, const_b=0.55)
        link(grain, g, 'A', 'R')
        base = mul(m, base, g, -450, 100)
    out(m, mul(m, base, col, -250, 0), MP.MP_BASE_COLOR)
    out(m, noise(m, 0.01, 0.72, 0.88, -450, 400, levels=3), MP.MP_ROUGHNESS)
    out(m, const(m, 0.45, -250, 500), MP.MP_SPECULAR)
    n = tex(m, '/Game/StarterContent/Textures/T_Ground_Gravel_N', uv, -1000, 650, normal=True)
    if n:
        out(m, n, MP.MP_NORMAL, 'RGB')
    finish(m, name)


def grass(name):
    m = new_mat(name)
    col = vec(m, 'Color', (1, 1, 1), -1000, -200)
    uv = world_uv(m, 260.0, -1400, 100)
    d = tex(m, '/Game/StarterContent/Textures/T_Ground_Grass_D', uv, -1000, 100)
    base = mulc(m, d, 0.85, -800, 100, aout='RGB') if d else const3(m, (0.06, 0.11, 0.03), -800, 100)
    # large-scale colour variation so the field does not tile
    tint = lerp(m, const3(m, (0.85, 1.0, 0.7), -800, 300), const3(m, (1.1, 0.95, 0.75), -800, 380), noise(m, 0.0015, 0, 1, -800, 460), -600, 350)
    out(m, mul(m, mul(m, base, tint, -450, 150), col, -250, 100), MP.MP_BASE_COLOR)
    out(m, const(m, 0.92, -250, 400), MP.MP_ROUGHNESS)
    n = tex(m, '/Game/StarterContent/Textures/T_Ground_Grass_N', uv, -1000, 600, normal=True)
    if n:
        out(m, n, MP.MP_NORMAL, 'RGB')
    finish(m, name)


def concrete(name):
    m = new_mat(name)
    col = vec(m, 'Color', (1, 1, 1), -1000, -200)
    uv = world_uv(m, 300.0, -1400, 100)
    d = tex(m, '/Game/StarterContent/Textures/T_Concrete_Poured_D', uv, -1000, 100)
    base = d if d else const3(m, (0.32, 0.31, 0.29), -800, 100)
    out(m, mul(m, base, col, -400, 50, aout='RGB' if d else ''), MP.MP_BASE_COLOR)
    out(m, const(m, 0.85, -400, 300), MP.MP_ROUGHNESS)
    n = tex(m, '/Game/StarterContent/Textures/T_Concrete_Poured_N', uv, -1000, 500, normal=True)
    if n:
        out(m, n, MP.MP_NORMAL, 'RGB')
    finish(m, name)


def painted(name):
    m = new_mat(name)
    col = vec(m, 'Color', (0.8, 0.8, 0.8), -800, 0)
    grime = noise(m, 0.02, 0.85, 1.0, -800, 200, levels=3)
    out(m, mul(m, col, grime, -500, 50), MP.MP_BASE_COLOR)
    out(m, scal(m, 'Rough', 0.6, -500, 250), MP.MP_ROUGHNESS)
    finish(m, name)


def metal(name):
    m = new_mat(name)
    out(m, vec(m, 'Color', (0.75, 0.75, 0.77), -600, 0), MP.MP_BASE_COLOR)
    out(m, const(m, 1.0, -600, 200), MP.MP_METALLIC)
    out(m, scal(m, 'Rough', 0.35, -600, 300), MP.MP_ROUGHNESS)
    finish(m, name)


for n, f in [('M_CarPaint', car_paint), ('M_Glass', glass), ('M_Trim', trim), ('M_Light', light), ('M_Tyre', tyre), ('M_Rim', rim),
             ('M_Asphalt', asphalt), ('M_Grass', grass), ('M_Concrete', concrete), ('M_Painted', painted), ('M_Metal', metal)]:
    build(n, f)

unreal.log('TT make_assets: made %d: %s' % (len(made), ', '.join(made)))
for f in failed:
    unreal.log_error('TT make_assets: FAILED ' + f)
print('TT_ASSETS_DONE made=%d failed=%d' % (len(made), len(failed)))
