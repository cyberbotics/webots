from geometry.vec3 import Vec3


def cubemap_lookup(direction):
    abs = direction.abs()
    ma = 0.0
    u = 0.0
    v = 0.0
    fi = 0
    if abs.z >= abs.x and abs.z >= abs.y:
        fi = 5 if direction.z < 0.0 else 4
        ma = 0.5 / abs.z
        u = -direction.x if direction.z < 0.0 else direction.x
        v = -direction.y
    elif abs.y >= abs.x:
        fi = 3 if direction.y < 0.0 else 2
        ma = 0.5 / abs.y
        u = direction.x
        v = -direction.z if direction.y < 0.0 else direction.z
    else:
        fi = 1 if direction.x < 0.0 else 0
        ma = 0.5 / abs.x
        u = direction.z if direction.x < 0.0 else -direction.z
        v = -direction.y
    return (u * ma + 0.5, v * ma + 0.5, fi)


def face_normals(i, x=0.0, y=0.0):
    if i == 0:
        return Vec3(-1.0, y, x).normalize()
    elif i == 1:
        return Vec3(1.0, y, x).normalize()
    elif i == 2:
        return Vec3(x, -1.0, y).normalize()
    elif i == 3:
        return Vec3(x, 1.0, y).normalize()
    elif i == 4:
        return Vec3(x, y, -1.0).normalize()
    else:  # i == 5
        return Vec3(x, y, 1.0).normalize()
