"""Explicit licensed backend; device processing lives in XRT_devices."""
from geo_kin_core.session import resolve_session

def make_session(**config):
    return resolve_session(robot="dual_kinova3", backend="licensed", **config)
