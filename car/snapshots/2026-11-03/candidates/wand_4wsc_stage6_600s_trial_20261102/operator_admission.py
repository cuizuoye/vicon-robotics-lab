# Appended into the private admission module by build.py. No hardware IO.
def verify_operator_trial(p):
    expected=dict(kind='EXPLICIT_UNQUALIFIED_600S_OPERATOR_REQUEST_V1',
        authorization_path=str(AUTHORIZATION_PATH),authorization_sha256=AUTHORIZATION_SHA,
        maximum_active_s=600.,stage=6,measurements_still_unknown=True,
        no_physical_qualification_claim=True,onsite_power_cut_operator_present=True)
    require(p.get('operator_duration_trial')==expected,'explicit unqualified duration trial declaration required')
    require(p.get('long_duration_review')=={},'trial must not masquerade as measured approval')
    raw=AUTHORIZATION_PATH.read_bytes()
    require(hashlib.sha256(raw).hexdigest()==AUTHORIZATION_SHA,'operator request record changed')
    r=json.loads(raw)
    require(r.get('operator_requested_unqualified_600s_trial') is True
        and r.get('site_confirmation_received') is True
        and r.get('physical_qualified') is False
        and r.get('electrical_thermal_verified') is False
        and r.get('physical_stop_envelope_verified') is False,
        'explicit request is not a physical qualification')
