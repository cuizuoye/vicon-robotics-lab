"""Used healthy run allowance is a preparation issue, never permission to move."""
def zero_preparation_ready(transport):
    return bool(transport.get('ready') is True or (
        transport.get('healthy_budget_used') is True
        and transport.get('native_disabled_zero') is True
        and transport.get('native_fault_bits') == 0))
