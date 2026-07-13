"""Minimal binary PLY/PCD XYZ readers shared by offline experiments."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def read_xyz_ply(path: Path) -> np.ndarray:
    """Read XYZ fields from a binary little-endian PLY vertex element."""
    with path.open("rb") as stream:
        vertex_count = 0
        format_name = ""
        properties: list[tuple[str, str]] = []
        in_vertex = False
        while True:
            line = stream.readline().decode("ascii").strip()
            if line.startswith("format "):
                format_name = line.split()[1]
            elif line.startswith("element vertex "):
                vertex_count = int(line.split()[2])
                in_vertex = True
            elif line.startswith("element "):
                in_vertex = False
            elif in_vertex and line.startswith("property "):
                _property, type_name, name = line.split()
                properties.append((name, type_name))
            elif line == "end_header":
                break
        if format_name != "binary_little_endian":
            raise RuntimeError(f"only binary_little_endian PLY is supported: {path}")
        formats = {"float": "<f4", "float32": "<f4", "double": "<f8"}
        try:
            dtype = np.dtype(
                [(name, formats[type_name]) for name, type_name in properties]
            )
        except KeyError as error:
            raise RuntimeError(
                f"unsupported PLY vertex property type: {error}"
            ) from error
        vertices = np.fromfile(stream, dtype=dtype, count=vertex_count)
    return np.column_stack((vertices["x"], vertices["y"], vertices["z"])).astype(
        np.float32, copy=False
    )


def read_xyz_pcd(path: Path) -> np.ndarray:
    """Read XYZ fields from an uncompressed binary PCD cloud."""
    with path.open("rb") as stream:
        header: dict[str, list[str]] = {}
        while True:
            tokens = stream.readline().decode("ascii").strip().split()
            if not tokens:
                continue
            header[tokens[0].upper()] = tokens[1:]
            if tokens[0].upper() == "DATA":
                break
        if header["DATA"][0].lower() != "binary":
            raise RuntimeError(f"only binary PCD is supported: {path}")
        fields = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        type_map = {
            ("F", 4): "<f4",
            ("F", 8): "<f8",
            ("I", 4): "<i4",
            ("U", 4): "<u4",
        }
        dtype_fields = []
        for name, size, type_name, count in zip(fields, sizes, types, counts):
            scalar = type_map[(type_name.upper(), size)]
            field = (name, scalar) if count == 1 else (name, scalar, (count,))
            dtype_fields.append(field)
        points = int(header.get("POINTS", header["WIDTH"])[0])
        cloud = np.fromfile(stream, dtype=np.dtype(dtype_fields), count=points)
    return np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(
        np.float32, copy=False
    )
