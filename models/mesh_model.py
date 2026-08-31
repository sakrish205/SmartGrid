"""Single source of truth: wraps MeshData and the current region classification."""
from __future__ import annotations
from typing import Optional
import numpy as np
from app.mesh.loader import load_mesh
from app.mesh.preprocessor import preprocess, MeshData
from app.mesh.regions import classify_regions


class MeshModel:
    def __init__(self) -> None:
        self._data: Optional[MeshData] = None
        self._regions: Optional[dict[str, np.ndarray]] = None

    @property
    def is_loaded(self) -> bool:
        return self._data is not None

    @property
    def data(self) -> Optional[MeshData]:
        return self._data

    @property
    def regions(self) -> Optional[dict[str, np.ndarray]]:
        return self._regions

    def load(self, filepath: str, up_axis: int = 2) -> MeshData:
        mesh = load_mesh(filepath)
        self._data = preprocess(mesh, filepath, up_axis=up_axis)
        self._regions = classify_regions(self._data)
        return self._data

    def get_region_faces(self, region_id: str) -> np.ndarray:
        if self._regions is None:
            return np.array([], dtype=np.int64)
        return self._regions.get(region_id, np.array([], dtype=np.int64))

    def get_selection_scalar_array(self, selected_face_ids: np.ndarray) -> np.ndarray:
        """Return per-face float array: 1.0 for selected faces, 0.0 for others."""
        if self._data is None:
            return np.array([])
        scalars = np.zeros(len(self._data.trimesh_mesh.faces), dtype=float)
        if len(selected_face_ids) > 0:
            scalars[selected_face_ids] = 1.0
        return scalars
