"""Shared input errors and numeric validation."""
import math


class InputError(ValueError):
    pass


def number(value, name, low, high):
    if isinstance(value, bool):
        raise InputError(f'{name} must be a number.')
    try:
        v = float(value)
    except (ValueError, TypeError):
        raise InputError(f'{name} must be a number.') from None
    if not math.isfinite(v) or not low <= v <= high:
        raise InputError(f'{name} must be between {low} and {high}.')
    return v
