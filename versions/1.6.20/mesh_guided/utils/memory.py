def estimated_sample_bytes(count, include_sh_rest=False):
    # Position, normal, UV, ids, barycentrics, colors, quaternions, scales and opacity.
    bytes_per_sample = 140 + (135 if include_sh_rest else 0)
    return int(max(0, count)) * bytes_per_sample


def fits_memory_budget(count, budget_mb, include_sh_rest=False):
    return estimated_sample_bytes(count, include_sh_rest) <= int(budget_mb) * 1024 * 1024

