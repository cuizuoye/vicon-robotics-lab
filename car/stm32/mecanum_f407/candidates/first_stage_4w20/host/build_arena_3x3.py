#!/usr/bin/env python3
"""Generate/build-source only: fixed 3x3m +0.50m buffer +1:1, 4W20 unchanged."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent


def edit(path,old,new,count=1):
    text=path.read_text()
    if text.count(old)!=count:raise RuntimeError('source drift: '+str(path)+' '+old)
    path.write_text(text.replace(old,new))


def generate(output,snapshot):
    spec=importlib.util.spec_from_file_location('one_to_one_generator',HERE/'build_one_to_one.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.generate(output,snapshot)
    control=output/'mecanum_wand_one_to_one_control/mecanum_wand_one_to_one_control'
    hardware=output/'mecanum_wand_one_to_one_hardware/mecanum_wand_one_to_one_hardware'
    for name in ('arena_geometry.py','arena_config.py'):
        (control/name).write_text((HERE/name).read_text())
    commission=hardware/'commissioning.py'
    edit(commission,'from mecanum_wand_one_to_one_control.safety_core import SafetyConfig, ZERO, fresh_envelope',
         'from mecanum_wand_one_to_one_control.safety_core import ZERO, fresh_envelope\n'
         'from mecanum_wand_one_to_one_control.arena_config import ArenaSafetyConfig as SafetyConfig')
    edit(commission,"'ground_z_m', 'half_length_m', 'half_width_m', 'boundary_margin_m')",
         "'ground_z_m', 'half_length_m', 'half_width_m', 'boundary_margin_m',\n"
         "                   'arena_origin_x_m', 'arena_origin_y_m', 'arena_heading_rad',\n"
         "                   'arena_forward_max_m', 'arena_left_max_m', 'arena_buffer_m')")
    edit(commission,"        if self.profile and self.profile.get('mapping_scale') != 1.0:",
         "        if self.profile.get('arena_revision') != 'fixed_corner_3x3_buffer050_v1':\n"
         "            raise ValueError('explicit fixed 3x3/0.50m arena profile required')\n"
         "        if mode == 'hardware':\n"
         "            required = ('extended_arena_and_buffer_clear_and_tracked',\n"
         "                        'saved_start_and_heading_match', 'immediate_power_cut_ready_now')\n"
         "            if any(self.profile.get('arena_confirmations', {}).get(k) is not True for k in required):\n"
         "                raise ValueError('ARENA_3X3_SITE_CONFIRMATION_REQUIRED')\n"
         "        if self.profile and self.profile.get('mapping_scale') != 1.0:")
    edit(commission,'if self.config.max_goal_radius_m > 3.0:', 'if self.config.max_goal_radius_m > 4.75:')
    edit(commission,"'acceptance target radius is at most 3m'","'acceptance target radius is at most 4.75m; goals still fixed to 3x3m'")
    edit(commission,'ONE_TO_ONE_020_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY',
         'ONE_TO_ONE_3X3_020_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY')
    edit(commission,'            self.config = SafetyConfig()',
         "            if not profile:\n"
         "                raise ValueError('explicit 3x3 arena profile required even for preview')\n"
         '            self.config = None')
    # Decouple nominal final-goal clipping from reference clipping. Both
    # supervisor and final gate still use the same unchanged timed authority.
    edit(control/'core.py','def __init__(self, config=None, goal_projector=None):',
         'def __init__(self, config=None, goal_projector=None, reference_projector=None):')
    edit(control/'core.py','        self.goal_projector = goal_projector',
         '        self.goal_projector = goal_projector\n'
         '        self.reference_projector = reference_projector or goal_projector')
    edit(control/'core.py',
         '        if self.goal_projector:\n            candidate = self.goal_projector(candidate, self.mapping.car_origin,',
         '        if self.reference_projector:\n            candidate = self.reference_projector(candidate, self.mapping.car_origin,')
    edit(control/'core.py','        self.mapped_goal = self.mapping.goal_from_local(*self.filtered_local)',
         '        self.mapped_goal = self.mapping.goal_from_local(*self.filtered_local)\n'
         '        if self.goal_projector is not None:\n'
         '            self.mapped_goal = self.goal_projector(self.mapped_goal, self.mapping.car_origin,\n'
         '                                                   self.config.max_goal_radius_m)')
    edit(control/'safety_core.py',
         "            self.c.project_goal if self.c.goal_boundary_policy == 'clamp' else None)",
         "            self.c.project_goal if self.c.goal_boundary_policy == 'clamp' else None,\n"
         "            reference_projector=getattr(self.c, 'project_reference', None))")
    edit(control/'safety_nodes.py','        values = {}\n        for f in fields(SafetyConfig):',
         '        config_type = type(runtime.config) if runtime else SafetyConfig\n'
         '        values = {}\n        for f in fields(config_type):')
    edit(control/'safety_nodes.py','self.core = core_type(SafetyConfig(**values))',
         'self.core = core_type(config_type(**values))')
    edit(control/'operator_guidance.py','    budget = min(budget, config.max_goal_radius_m)',
         "    if hasattr(config, 'goal_clearance_radius'):\n"
         "        budget = config.goal_clearance_radius(car_origin)\n"
         '    budget = min(budget, config.max_goal_radius_m)')
    # Draw true oriented polygons, not the larger world-axis bounding box.
    ui=control/'operator_console.py';text=ui.read_text()
    start=text.index('        c.create_rectangle(*self.map_point(self.config.arena_x_min,')
    end=text.index('        points = [v for x, y in self.node.trace',start)
    text=text[:start]+'''        def polygon(vertices, **style):
            points = [v for x,y in vertices for v in self.map_point(x,y)]
            c.create_polygon(*points, fill='', **style)
        polygon(self.config.fence_polygon(), outline='#d85d5d', width=2)
        polygon(self.config.goal_polygon(), outline='#319a6a', width=2)
        if self.wand_reference_margin:
            polygon(self.config.rod_reference_polygon(), outline='#9c79c4', width=2, dash=(7,4))
        c.create_text(12,14,anchor='w',text='绿：固定3×3m车中心目标区；红：车体外缘+50cm物理缓冲外框',
                      fill='#8b4c4c',font=('Noto Sans CJK SC',9))
        c.create_text(12,32,anchor='w',text='橙虚线：保留20cm余量的硬停中心边界；紫：杆参考区，非已验证覆盖',
                      fill='#77529d',font=('Noto Sans CJK SC',9))
''' + text[end:]
    start=text.index('                xmin, xmax, ymin, ymax = center_bounds(self.config, p.yaw)')
    end=text.index('            corners = []',start)
    text=text[:start]+'''                polygon(self.config.stop_center_polygon(p.yaw), outline='#db8a28', width=2, dash=(5,3))
''' + text[end:]
    text=text.replace('水平位移1:1；','水平位移1:1；3×3m目标区，四边50cm缓冲；')
    ui.write_text(text)
    # The 1:1 builder's old-area profile was made in THIS fresh output only.
    config=output/'mecanum_wand_one_to_one_hardware/config'
    old=config/'one_to_one.CURRENT_AREA.SITE_CONFIRMATION_REQUIRED.json'
    profile=json.loads(old.read_text());old.unlink()
    s=profile['safety'];origin=profile['evidence']['preview_start_xy_m'];yaw=profile['evidence']['car_heading_rad']
    s.update(arena_origin_x_m=origin[0],arena_origin_y_m=origin[1],arena_heading_rad=yaw,
        arena_forward_max_m=3.,arena_left_max_m=3.,arena_buffer_m=.50,max_goal_radius_m=4.75)
    c,sn=math.cos(yaw),math.sin(yaw)
    corners=[(origin[0]+c*x-sn*y,origin[1]+sn*x+c*y)
             for x in (-.595,3.595) for y in (-.605,3.605)]
    s.update(arena_x_min=min(x for x,y in corners),arena_x_max=max(x for x,y in corners),
             arena_y_min=min(y for x,y in corners),arena_y_max=max(y for x,y in corners))
    profile['arena_revision']='fixed_corner_3x3_buffer050_v1'
    profile['arena_confirmations']=dict(extended_arena_and_buffer_clear_and_tracked=False,
        saved_start_and_heading_match=False,immediate_power_cut_ready_now=False)
    profile['confirmations']['fixed_arena_checked']=False
    profile['evidence']['nominal_center_workspace'].update(forward_max_m=3,left_max_m=3)
    profile['evidence']['change_scope']='Fixed oriented 3x3m goals, separate body+0.50m physical fence, 1:1; 4W20 unchanged'
    profile['evidence']['site_scope']='Extended software geometry requested, NOT a new Vicon coverage or stop-distance measurement'
    profile['input_workspace']['mapping']='1:1 XY; fixed oriented 3x3m final goals; independent hard-stop footprint fence'
    profile['arena_buffer_policy']=dict(physical_buffer_outside_reference_body_m=.50,
        preserved_extra_margin_m=.20,hard_center_range_at_reference_heading_m=[-.30,3.30],
        buffer_is_not_an_extra_goal_region=True,physical_stop_distance_measured=False,
        physical_clear_extent_m=[4.19,4.21],all_yaw_clear_extent_planning_m=[4.30,4.30])
    target=config/'one_to_one.3x3_BUFFER050.SITE_CONFIRMATION_REQUIRED.json'
    target.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    # Historical planning artifact remains clearly archived, never a profile.
    report=dict(arena_3x3_implemented=True,mapping_scale=1.,buffer_m=.50,
        preserved_extra_margin_m=.20,max_goal_radius_m=4.75,domain=97,
        namespace='/wand_one_to_one_acceptance',firmware_changed=False,hardware_started=False,
        geometry_fixed_in_world=True,site_approved=False,profile=str(target.resolve()),
        sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    (output/'generation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    for package in ('mecanum_wand_one_to_one_control','mecanum_wand_one_to_one_hardware'):
        (output/package/'README.md').write_text(
            '# Fixed 3x3m / 0.50m buffer /1:1 software\n\n'
            '4W20 unchanged;0.20m/s vector;35%PWM. Domain97; /wand_one_to_one_acceptance.\n'
            'Oriented goal box uses saved original start/heading; preparing NEVER recenters it.\n'
            'Physical buffer is measured beyond reference car outline; retained0.20m margin trips earlier.\n'
            'Fresh extended-area/Vicon/power-cut confirmations required before hardware transport.\n'
            'Braking/current/thermal acceptance not measured; no automatic motion or new firmware.\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--accepted-snapshot',required=True,type=Path)
    a=p.parse_args();generate(a.output.resolve(),a.accepted_snapshot)
