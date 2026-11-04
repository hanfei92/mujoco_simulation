import os
import trimesh

for f in os.listdir('.'):
    if f.endswith('.STL') or f.endswith('.stl'):
        try:
            m = trimesh.load(f, force='mesh')
            n_faces = len(m.faces)
            # 如果面数超限，简化
            if n_faces > 200000:
                m = m.simplify_quadric_decimation(200000)
                n_faces = len(m.faces)
            # 导出为二进制 STL
            m.export(f, file_type='stl')
            print(f'OK: {f}  faces={n_faces}')
        except Exception as e:
            print(f'FAIL: {f}: {e}')