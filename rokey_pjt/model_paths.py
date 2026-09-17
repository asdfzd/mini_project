"""Resolve packaged model artifacts without machine-specific absolute paths."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


PACKAGE_NAME = "rokey_pjt"
DEFAULT_MODEL_NAME = "my_best_v2.pt"


def get_default_model_path() -> str:
    """Return the installed package model, with a source-tree fallback for development."""
    try:
        package_share = Path(get_package_share_directory(PACKAGE_NAME))
        return str(package_share / "models" / DEFAULT_MODEL_NAME)
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1] / "models" / DEFAULT_MODEL_NAME)


def resolve_model_path(configured_path: str) -> str:
    """Resolve a ROS parameter value, using the packaged default when it is empty."""
    if configured_path:
        return str(Path(configured_path).expanduser())
    return get_default_model_path()


__all__ = ["DEFAULT_MODEL_NAME", "get_default_model_path", "resolve_model_path"]
