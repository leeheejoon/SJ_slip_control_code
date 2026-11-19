"""Generate optional RH56E2 collision hulls (requires numpy and scipy).

Visual meshes remain unchanged. GLB scene transforms are baked into STL.
0.3 mm grid rounding removes tiny CAD details (vertex displacement <=0.26 mm).
Convex hulls fill concavities; runtime contact behavior still needs validation.
"""
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
MESHES = ROOT / "meshes" / "RH56E2"
OUTPUT = MESHES / "collision"


def glb_points(path):
    data = path.read_bytes()
    offset = 12
    chunks = {}
    while offset < len(data):
        length, kind = struct.unpack_from("<II", data, offset)
        chunks[kind] = data[offset + 8:offset + 8 + length]
        offset += 8 + length
    doc = json.loads(chunks[0x4E4F534A])
    binary = chunks[0x004E4942]

    def positions(index):
        acc = doc["accessors"][index]
        assert acc["componentType"] == 5126 and acc["type"] == "VEC3"
        view = doc["bufferViews"][acc["bufferView"]]
        start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
        return np.ndarray((acc["count"], 3), dtype="<f4", buffer=binary,
                          offset=start, strides=(view.get("byteStride", 12), 4)).copy()

    result = []

    def visit(index, parent):
        node = doc["nodes"][index]
        if "matrix" in node:
            local = np.array(node["matrix"]).reshape(4, 4, order="F")
        else:
            x, y, z, w = node.get("rotation", [0, 0, 0, 1])
            rotation = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            local = np.eye(4)
            local[:3, :3] = rotation @ np.diag(node.get("scale", [1, 1, 1]))
            local[:3, 3] = node.get("translation", [0, 0, 0])
        transform = parent @ local
        if "mesh" in node:
            for primitive in doc["meshes"][node["mesh"]]["primitives"]:
                pts = positions(primitive["attributes"]["POSITION"])
                result.append(pts @ transform[:3, :3].T + transform[:3, 3])
        for child in node.get("children", []):
            visit(child, transform)

    for index in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        visit(index, np.eye(4))
    return np.concatenate(result)


def stl_points(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) == 84 + count * 50:
        dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)),
                          ("attribute", "<u2")])
        return np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)
    return np.array([list(map(float, line.split()[1:]))
                     for line in data.decode().splitlines()
                     if line.strip().startswith("vertex ")])


def main():
    model = ET.parse(ROOT / "xacro" / "hands" / "RH56E2.xacro")
    sources = {m.get("filename").split("/meshes/RH56E2/")[1]
               for m in model.findall(".//collision//mesh")
               if "/RH56E2/collision/" not in m.get("filename")}
    OUTPUT.mkdir(exist_ok=True)
    for relative in sorted(sources):
        source = MESHES / relative
        original = glb_points(source) if source.suffix == ".glb" else stl_points(source)
        points = np.unique(np.round(original / 0.0003) * 0.0003, axis=0)
        hull = ConvexHull(points)
        target = OUTPUT / (source.stem + ".stl")
        with target.open("wb") as stream:
            stream.write(b"RH56E2 optional convex collision hull".ljust(80, b"\0"))
            stream.write(struct.pack("<I", len(hull.simplices)))
            for face, equation in zip(hull.simplices, hull.equations):
                vertices = points[face].copy()
                normal = np.cross(vertices[1]-vertices[0], vertices[2]-vertices[0])
                if np.dot(normal, equation[:3]) < 0:
                    vertices[[1, 2]] = vertices[[2, 1]]
                stream.write(struct.pack("<12fH", *equation[:3], *vertices.ravel(), 0))
        assert np.max(np.abs(original.min(0)-points.min(0))) <= 0.000151
        assert np.max(np.abs(original.max(0)-points.max(0))) <= 0.000151
        print(f"{source.name}: {len(hull.simplices)} hull triangles")


if __name__ == "__main__":
    main()
