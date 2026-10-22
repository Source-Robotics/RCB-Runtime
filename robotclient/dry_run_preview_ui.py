"""NiceGUI controls for inspecting a saved dry-run plan without robot I/O."""

import os
from pathlib import Path
import json
import time

import numpy as np
from nicegui import ui

from .dry_run_preview import ACTIVE_PREVIEW_PATH, load_dry_run_preview


JOINT_COLORS = ('#2563eb', '#059669', '#d97706', '#dc2626', '#0891b2', '#7c3aed')
TCP_LABELS = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ')
METRICS = {
    'joint_position_rad': ('Joint Position', 'rad', 'joint_position_rad',
                           tuple(f'J{i + 1}' for i in range(6))),
    'joint_velocity_rad_s': ('Joint Velocity', 'rad/s', 'joint_velocity_rad_s',
                             tuple(f'J{i + 1}' for i in range(6))),
    'joint_acceleration_rad_s2': ('Joint Acceleration', 'rad/s^2',
                                   'joint_acceleration_rad_s2',
                                   tuple(f'J{i + 1}' for i in range(6))),
    'joint_jerk_rad_s3': ('Joint Jerk', 'rad/s^3', 'joint_jerk_rad_s3',
                          tuple(f'J{i + 1}' for i in range(6))),
    'tcp_pose': ('TCP Pose', 'm / rad', 'tcp_pose', TCP_LABELS),
    'tcp_velocity': ('TCP Velocity', 'm/s / rad/s', 'tcp_velocity', TCP_LABELS),
    'tcp_acceleration': ('TCP Acceleration', 'm/s^2 / rad/s^2',
                         'tcp_acceleration', TCP_LABELS),
}

RAW_WINDOW_DURATION_S = 20.0
RAW_WINDOW_SHIFT_S = 4.0


def raw_window_indices(time_s, start_s, duration_s=RAW_WINDOW_DURATION_S):
    """Return a clamped time window and its half-open raw-sample index range."""
    time_s = np.asarray(time_s, dtype=np.float64)
    if time_s.ndim != 1 or not len(time_s):
        raise ValueError("time_s must be a non-empty one-dimensional array")
    first_time = float(time_s[0])
    last_time = float(time_s[-1])
    span = min(max(float(duration_s), 0.0), max(0.0, last_time - first_time))
    window_start = float(np.clip(start_s, first_time, max(first_time, last_time - span)))
    window_end = min(last_time, window_start + span)
    first_index = int(np.searchsorted(time_s, window_start, side="left"))
    last_index = int(np.searchsorted(time_s, window_end, side="right"))
    return window_start, window_end, first_index, max(first_index + 1, last_index)


def add_dry_run_preview(is_visible=None, angle_unit=None):
    """Add a self-contained static/scrub/playback viewer to the current UI context.

    angle_unit (2026-10-21): optional callable returning 'rad' or 'deg', the unit angles are SHOWN in (the
    dashboard's Angles toggle). The saved preview stays in radians; the plot, the readout and the tables convert.
    Returns a function that redraws everything in the current unit — call it after the unit changes."""
    visible = is_visible or (lambda: True)
    state = {
        'data': None, 'index': 0, 'playing': False, 'speed': 1.0,
        'last_wall_s': None, 'metric': 'joint_position_rad', 'axes': {0},
        'path_mode': 'xy', 'window_start_s': 0.0, 'window_first_index': 0,
        'window_dirty': False, 'active_mtime_ns': None,
    }
    axis_checkboxes = []

    def in_degrees():
        return angle_unit is not None and angle_unit() == 'deg'

    def unit_text(text):
        """A unit label in the display unit: 'rad/s^2' -> 'deg/s^2' while showing degrees."""
        return text.replace('rad', 'deg') if in_degrees() else text

    def angle_scale(metric, axis):
        """Factor from the saved value to the display unit: every joint axis is an angle, and so are the TCP's
        rotation components (axes 3..5); the TCP's x, y, z (metres) never convert."""
        if not in_degrees() or (metric.startswith('tcp_') and axis < 3):
            return 1.0
        return 180.0 / np.pi

    def project(x, y, z):
        mode = state['path_mode']
        if mode == 'xy':
            return x, y
        if mode == 'xz':
            return x, z
        if mode == 'yz':
            return y, z
        return x - y, z + 0.5 * (x + y)

    def path_labels():
        return {
            'xy': ('X', 'Y'), 'xz': ('X', 'Z'), 'yz': ('Y', 'Z'),
            'iso': ('X-Y', 'Z + 0.5(X+Y)'),
        }[state['path_mode']]

    def signal_axes():
        return tuple(sorted(state['axes']))

    def axis_unit(axis):
        if not state['metric'].startswith('tcp_'):
            return unit_text(METRICS[state['metric']][1])
        if state['metric'] == 'tcp_pose':
            return 'm' if axis < 3 else unit_text('rad')
        if state['metric'] == 'tcp_velocity':
            return 'm/s' if axis < 3 else unit_text('rad/s')
        return 'm/s^2' if axis < 3 else unit_text('rad/s^2')

    def signal_unit():
        units = {axis_unit(axis) for axis in signal_axes()}
        return units.pop() if len(units) == 1 else 'mixed units'


    def _number_text(value):
        """Format a numeric diagnostic value without exposing JSON sentinels."""
        if value is None:
            return '-'
        try:
            number = float(value)
        except (TypeError, ValueError):
            return str(value)
        if not np.isfinite(number):
            return 'inf' if number > 0 else '-'
        return f'{number:.3g}'

    def _diagnostic_value(details, *keys):
        for key in keys:
            if details.get(key) is not None:
                return details[key]
        return None

    def _diagnostic_text(segment):
        details = segment.get('diagnostic') or {}
        notes = []
        zones = segment.get('blend_zones') or []
        if zones:
            notes.append(f'{len(zones)} blend zone' + ('s' if len(zones) != 1 else ''))
        if details.get('success') is False:
            notes.append('FAILED')
        if details.get('singularity_warning'):
            notes.append('SINGULARITY')
        if details.get('fk_warning'):
            notes.append('FK warning')
        if details.get('branch_jump_indices'):
            notes.append('branch jump')
        if details.get('ik_failures'):
            notes.append('IK failure')
        condition = _diagnostic_value(details, 'max_jacobian_condition', 'jacobian_condition')
        sigma = _diagnostic_value(details, 'min_jacobian_sigma', 'jacobian_sigma_min')
        if condition is not None:
            notes.append(f'cond={_number_text(condition)}')
        if sigma is not None:
            notes.append(f'sigma={_number_text(sigma)}')
        return '; '.join(notes) if notes else 'OK'

    def diagnostic_rows(data):
        """Build one read-only summary row per saved planned segment."""
        rows = []
        offset = 0
        k = angle_scale('joint_position_rad', 0)        # the peaks below are joint rates
        for index, segment in enumerate(data.segments):
            count = int(data.segment_sample_counts[index])
            stop = min(offset + count, len(data.time_s))
            if stop <= offset:
                offset = stop
                continue
            q = data.joint_position_rad[offset:stop]
            qd = data.joint_velocity_rad_s[offset:stop]
            qdd = data.joint_acceleration_rad_s2[offset:stop]
            qddd = data.joint_jerk_rad_s3[offset:stop]
            end_q_deg = np.rad2deg(q[-1])
            end_tcp_mm = data.tcp_pose[stop - 1, :3] * 1000.0
            details = segment.get('diagnostic') or {}
            rows.append({
                'sequence': f"{int(segment.get('sequence', index + 1)):02d}",
                'command': str(segment.get('command', 'unknown')),
                'profile': str(segment.get('profile', '-')),
                'duration': f"{float(segment.get('end_time_s', 0.0)) - float(segment.get('start_time_s', 0.0)):.3f}",
                'samples': count,
                'end_joints': ', '.join(f'{value:.2f}' for value in end_q_deg),
                'end_tcp': ', '.join(f'{value:.2f}' for value in end_tcp_mm),
                'peak_velocity': f'{float(np.max(np.abs(qd))) * k:.3f}',
                'peak_acceleration': f'{float(np.max(np.abs(qdd))) * k:.3f}',
                'peak_jerk': f'{float(np.max(np.abs(qddd))) * k:.3f}',
                'sigma_min': _number_text(_diagnostic_value(
                    details, 'min_jacobian_sigma', 'jacobian_sigma_min')),
                'condition': _number_text(_diagnostic_value(
                    details, 'max_jacobian_condition', 'jacobian_condition')),
                'status': _diagnostic_text(segment),
            })
            offset = stop
        return rows

    def update_diagnostics():
        data = state['data']
        diagnostic_table.rows = [] if data is None else diagnostic_rows(data)
        diagnostic_table.update()
    def active_window():
        """Current raw 4 ms plotting window as times and half-open sample indices."""
        data = state['data']
        if data is None:
            return 0.0, 0.0, 0, 0
        return raw_window_indices(data.time_s, state['window_start_s'])
    def signal_tooltip_formatter():
        """Show the exact saved sample and selected signal values at the cursor."""
        _, _, _, labels = METRICS[state['metric']]
        units = {labels[axis]: axis_unit(axis) for axis in signal_axes()}
        first_index = int(state['window_first_index'])
        return f'''function(params) {{
            const point = params.find(item => Array.isArray(item.value));
            if (!point) return '';
            const time = Number(point.value[0]);
            let html = '<b>sample ' + ({first_index} + point.dataIndex + 1) + '</b>   t=' + time.toFixed(3) + ' s';
            const units = {json.dumps(units)};
            params.forEach(function(item) {{
                if (!Array.isArray(item.value)) return;
                const value = Number(item.value[1]);
                const unit = units[item.seriesName] || '';
                html += '<br/>' + item.marker + item.seriesName + ': ' + value.toPrecision(7) + (unit ? ' ' + unit : '');
            }});
            return html;
        }}'''

    def signal_options():
        title, _, _, labels = METRICS[state['metric']]
        selected = signal_axes()
        label = 'All axes' if len(selected) == 6 else ', '.join(labels[index] for index in selected)
        unit = signal_unit()
        return {
            'animation': False,
            'title': {'text': f'{title}: {label}', 'left': 8, 'top': 2,
                      'textStyle': {'fontSize': 13, 'fontWeight': 'normal'}},
            'tooltip': {
                'trigger': 'axis', 'confine': True,
                'axisPointer': {'type': 'cross', 'snap': True, 'animation': False},
                ':formatter': signal_tooltip_formatter(),
            },
            'legend': {'data': [labels[index] for index in selected], 'top': 4, 'left': 'center'},
            'grid': {'left': '13%', 'right': '4%', 'top': 58, 'bottom': 42},
            'dataZoom': [
                {'type': 'inside', 'xAxisIndex': [0, 1], 'filterMode': 'none',
                 'start': 0, 'end': 100, 'zoomOnMouseWheel': True,
                 'moveOnMouseMove': True, 'moveOnMouseWheel': False},
                {'type': 'slider', 'xAxisIndex': [0, 1], 'height': 14, 'bottom': 3,
                 'filterMode': 'none', 'start': 0, 'end': 100},
            ],
            'xAxis': [
                {'type': 'value', 'name': 'planned time (s)', 'nameLocation': 'end'},
                {'type': 'value', 'position': 'top', 'name': 'sample #',
                 'nameLocation': 'end', 'min': 1, 'max': 1, 'minInterval': 1,
                 ':axisLabel.formatter': 'function(value) { return String(Math.round(value)); }'},
            ],
            'yAxis': {
                'type': 'value', 'name': f'{label} [{unit}]', 'nameLocation': 'middle',
                'nameGap': 48, 'scale': True,
            },
            'series': [
                {'name': labels[index], 'type': 'line', 'data': [], 'showSymbol': False,
                 'itemStyle': {'color': JOINT_COLORS[index]},
                 'lineStyle': {'width': 1.8}}
                for index in range(6)
            ],
        }

    def path_options():
        x_label, y_label = path_labels()
        return {
            'animation': False,
            'tooltip': {'trigger': 'axis', 'axisPointer': {'type': 'cross', 'animation': False}},
            'legend': {'data': ['Planned TCP', 'Selected sample'], 'top': 4, 'left': 'center'},
            'grid': {'left': '9%', 'right': '5%', 'top': 38, 'bottom': 30},
            'xAxis': {'type': 'value', 'name': f'{x_label} (m)', 'scale': True,
                      'nameLocation': 'end'},
            'yAxis': {'type': 'value', 'name': f'{y_label} (m)', 'scale': True,
                      'nameLocation': 'end'},
            'series': [
                {'name': 'Planned TCP', 'type': 'line', 'data': [], 'showSymbol': False,
                 'itemStyle': {'color': '#2563eb'}, 'lineStyle': {'width': 1.8}},
                {'name': 'Selected sample', 'type': 'scatter', 'data': [], 'symbolSize': 11,
                 'itemStyle': {'color': '#dc2626'}},
            ],
        }

    def path_bounds(points):
        """Return equal X/Y data spans so one metre looks like one metre."""
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        if not xs or not ys:
            return None, None, None, None
        center_x = (min(xs) + max(xs)) * 0.5
        center_y = (min(ys) + max(ys)) * 0.5
        half_span = max(max(xs) - min(xs), max(ys) - min(ys), 0.001) * 0.55
        return (center_x - half_span, center_x + half_span,
                center_y - half_span, center_y + half_span)

    def signal_bounds(values):
        """Return a finite padded range that frames the selected signal."""
        values = np.asarray(values, dtype=np.float64)
        low = float(np.min(values))
        high = float(np.max(values))
        span = high - low
        padding = max(span * 0.08, max(abs(low), abs(high), 1.0) * 1e-4)
        return low - padding, high + padding

    def event_label(event):
        """Create a compact timeline label from a saved virtual event."""
        command = str(event.get('command', 'event')).replace('_', ' ')
        details = event.get('details') or {}
        if details.get('kind') == 'io':
            state_text = 'ON' if details.get('value') else 'OFF'
            return f"I/O DO{details.get('pin', '?')} {state_text}"
        if details.get('immediate'):
            value = details.get('profile', details.get('percent', details.get('solver', details.get('policy'))))
            return f"setup {command}" if value is None else f"setup {command}={value}"
        if command.startswith('gripper') or command == 'move gripper':
            return command
        return command

    def add_timeline_annotations(data, series, view_start, view_end):
        """Attach only annotations intersecting the active raw plotting window."""
        areas = []
        markers = {}
        for index, segment in enumerate(data.segments):
            start = float(segment.get('start_time_s', 0.0))
            end = float(segment.get('end_time_s', start))
            if end < view_start or start > view_end:
                continue
            command = str(segment.get('command', 'motion'))
            profile = str(segment.get('profile', '')).strip()
            label = f"{segment.get('sequence', '?'):02d} {command}" + (f" [{profile}]" if profile else "")
            tint = ('rgba(217, 119, 6, 0.16)' if command == 'delay' else
                    ('rgba(37, 99, 235, 0.07)' if index % 2 == 0 else 'rgba(5, 150, 105, 0.07)'))
            if end > start:
                areas.append([
                    {'name': label, 'xAxis': start, 'itemStyle': {'color': tint}},
                    {'xAxis': end},
                ])
            for zone in segment.get('blend_zones', ()):
                zone_start = float(zone.get('start_time_s', start))
                zone_end = float(zone.get('end_time_s', zone_start))
                if zone_end < view_start or zone_start > view_end:
                    continue
                zone_kind = str(zone.get('kind', 'zone'))
                zone_label = 'Ruckig corner' if zone_kind == 'ruckig_corner' else 'blend corner'
                zone_color = ('rgba(190, 24, 93, 0.20)' if zone_kind == 'ruckig_corner'
                              else 'rgba(124, 58, 237, 0.20)')
                if zone_end > zone_start:
                    areas.append([
                        {'name': f"{zone_label} {zone.get('corner_index', '?')}",
                         'xAxis': zone_start, 'itemStyle': {'color': zone_color}},
                        {'xAxis': zone_end},
                    ])
            if view_start <= start <= view_end:
                marker = markers.setdefault(round(start, 9), {'starts': [], 'events': []})
                marker['starts'].append(label)

        for event in data.events:
            time_s = round(float(event.get('time_s', 0.0)), 9)
            if view_start <= time_s <= view_end:
                marker = markers.setdefault(time_s, {'starts': [], 'events': []})
                marker['events'].append(event_label(event))

        lines = []
        for time_s, marker in markers.items():
            for label in marker['starts']:
                sequence = label.split(maxsplit=1)[0]
                lines.append({
                    'name': f'{label} start', 'xAxis': time_s,
                    'lineStyle': {'color': '#0f4c81', 'type': 'solid', 'width': 2},
                    'label': {
                        'show': True, 'formatter': f'{sequence} start',
                        'color': '#0f4c81', 'fontSize': 10, 'fontWeight': 'bold',
                        'position': 'insideEndTop', 'rotate': 0, 'offset': [5, 0],
                        'padding': [2, 4], 'backgroundColor': 'rgba(239,246,255,0.94)',
                        'borderColor': '#93c5fd', 'borderWidth': 1, 'borderRadius': 2,
                    },
                })
            for label in marker['events']:
                lines.append({
                    'name': label, 'xAxis': time_s,
                    'lineStyle': {'color': '#7c3aed', 'type': 'dashed', 'width': 1.5},
                    'label': {
                        'show': True, 'formatter': label,
                        'color': '#6d28d9', 'fontSize': 10, 'lineHeight': 12,
                        'position': 'insideEndTop', 'rotate': 0, 'offset': [5, 18],
                        'padding': [2, 4], 'backgroundColor': 'rgba(255,255,255,0.90)',
                        'borderRadius': 2,
                    },
                })

        if areas:
            series['markArea'] = {'silent': True, 'label': {'show': False}, 'data': areas}
        if lines:
            series['markLine'] = {'silent': True, 'symbol': ['none', 'none'], 'data': lines}

    def segment_name(time_s):
        for segment in state['data'].segments:
            if segment.get('start_time_s', 0.0) <= time_s <= segment.get('end_time_s', 0.0):
                profile = str(segment.get('profile', '')).strip()
                suffix = f" [{profile}]" if profile else ""
                return f"{segment.get('sequence', '?'):02d} {segment.get('command', 'unknown')}{suffix}"
        return 'transition'

    def update_sample():
        data = state['data']
        if data is None:
            return
        index = state['index']
        time_s = float(data.time_s[index])
        metric = state['metric']
        title, _, attribute, labels = METRICS[metric]
        values = getattr(data, attribute)
        selected = signal_axes()
        if len(selected) == 1:
            axis = selected[0]
            value_text = (f"{labels[axis]}={float(values[index, axis]) * angle_scale(metric, axis):.6g} "
                          f"{signal_unit()}")
        else:
            value_text = ', '.join(
                f"{labels[axis]}={float(values[index, axis]) * angle_scale(metric, axis):.6g}" for axis in selected)
        sample_label.text = (
            f"sample {index + 1:,}/{len(data.time_s):,}   t={time_s:.3f} s   "
            f"{title}: {value_text}   {segment_name(time_s)}")
        kj = [angle_scale('joint_position_rad', axis) for axis in range(6)]
        kt = [angle_scale('tcp_pose', axis) for axis in range(6)]
        sample_table.rows = [{
            'axis': f'J{axis + 1} / {TCP_LABELS[axis]}',
            'joint_position': round(float(data.joint_position_rad[index, axis]) * kj[axis], 6),
            'joint_velocity': round(float(data.joint_velocity_rad_s[index, axis]) * kj[axis], 6),
            'joint_acceleration': round(float(data.joint_acceleration_rad_s2[index, axis]) * kj[axis], 6),
            'joint_jerk': round(float(data.joint_jerk_rad_s3[index, axis]) * kj[axis], 6),
            'tcp_pose': round(float(data.tcp_pose[index, axis]) * kt[axis], 6),
            'tcp_velocity': round(float(data.tcp_velocity[index, axis]) * kt[axis], 6),
            'tcp_acceleration': round(float(data.tcp_acceleration[index, axis]) * kt[axis], 6),
        } for axis in range(6)]
        sample_table.update()

    def draw_signal():
        data = state['data']
        signal_chart.options.clear()
        signal_chart.options.update(signal_options())
        if data is not None:
            view_start, view_end, first_index, last_index = active_window()
            state['window_first_index'] = first_index
            window_time = data.time_s[first_index:last_index]
            _, _, attribute, _ = METRICS[state['metric']]
            values = getattr(data, attribute)
            window_values = values[first_index:last_index] * np.array(
                [angle_scale(state['metric'], axis) for axis in range(values.shape[1])])
            selected = signal_axes()
            time_padding_s = max(0.02, (view_end - view_start) * 0.015)
            signal_chart.options['xAxis'][0].update({
                'min': view_start - time_padding_s,
                'max': view_end + time_padding_s,
            })
            signal_chart.options['xAxis'][1].update({
                'min': first_index + 1,
                'max': last_index,
            })
            for axis in selected:
                signal_chart.options['series'][axis]['data'] = np.column_stack((
                    np.round(window_time, 6), np.round(window_values[:, axis], 7))).tolist()
            y_min, y_max = signal_bounds(window_values[:, list(selected)])
            signal_chart.options['yAxis'].update({'min': y_min, 'max': y_max})
            add_timeline_annotations(data, signal_chart.options['series'][selected[0]],
                                     view_start, view_end)
            window_label.text = (
                f"Raw 4 ms view: {view_start:.3f} -> {view_end:.3f} s  "
                f"({last_index - first_index:,} samples)")
        signal_chart.update()
        if data is not None:
            update_sample()

    def draw_path():
        data = state['data']
        path_chart.options.clear()
        path_chart.options.update(path_options())
        if data is not None:
            _, _, first_index, last_index = active_window()
            projected = [
                [round(float(x), 6), round(float(y), 6)]
                for x, y in (project(*pose[:3]) for pose in data.tcp_pose[first_index:last_index])
            ]
            path_chart.options['series'][0]['data'] = projected
            selected_pose = data.tcp_pose[state['index'], :3]
            path_chart.options['series'][1]['data'] = [
                [round(float(x), 6), round(float(y), 6)]
                for x, y in [project(*selected_pose)]
            ] if first_index <= state['index'] < last_index else []
            x_min, x_max, y_min, y_max = path_bounds(projected)
            path_chart.options['xAxis'].update({'min': x_min, 'max': x_max})
            path_chart.options['yAxis'].update({'min': y_min, 'max': y_max})
        path_chart.update()

    def draw_all():
        draw_signal()
        draw_path()

    def set_axis_enabled(axis, enabled):
        """Update the plotted-axis set while keeping at least one trace visible."""
        selected = set(state['axes'])
        if enabled:
            selected.add(axis)
        elif len(selected) > 1:
            selected.remove(axis)
        else:
            axis_checkboxes[axis].value = True
            axis_checkboxes[axis].update()
            return
        state['axes'] = selected
        draw_signal()

    def set_window_start(start_s, redraw=True):
        """Move the raw plotting window and keep the selected sample visible."""
        data = state['data']
        if data is None:
            return
        state['window_start_s'] = float(start_s)
        _, _, first_index, last_index = active_window()
        if state['index'] < first_index:
            state['index'] = first_index
        elif state['index'] >= last_index:
            state['index'] = last_index - 1
        if redraw:
            draw_all()

    def set_index(index):
        data = state['data']
        if data is None:
            return
        state['index'] = max(0, min(int(index), len(data.time_s) - 1))
        view_start, view_end, _, _ = active_window()
        sample_time = float(data.time_s[state['index']])
        if sample_time < view_start or sample_time > view_end:
            set_window_start(sample_time - RAW_WINDOW_DURATION_S * 0.25)
        else:
            update_sample()

    def load_preview():
        path = path_input.value.strip()
        try:
            data = load_dry_run_preview(path)
        except (OSError, ValueError) as error:
            state.update(data=None, playing=False, last_wall_s=None)
            status.text = f"Preview load failed: {error}"
            status.classes(replace="text-sm text-negative")
            return
        state.update(data=data, index=0, playing=False, last_wall_s=None,
                     window_start_s=float(data.time_s[0]), window_first_index=0, window_dirty=False)
        try:
            state["active_mtime_ns"] = (ACTIVE_PREVIEW_PATH.stat().st_mtime_ns
                                        if Path(path) == ACTIVE_PREVIEW_PATH else None)
        except OSError:
            state["active_mtime_ns"] = None
        status.text = (f"Loaded {os.path.basename(path)}: {len(data.segments)} segments, "
                       f"{len(data.events)} events, {len(data.time_s):,} samples, "
                       f"{data.duration_s:.3f} s")
        status.classes(replace="text-sm text-positive")
        play_button.props("icon=play_arrow")
        draw_all()
        update_diagnostics()
    def toggle_play():
        if state['data'] is None:
            return
        state['playing'] = not state['playing']
        state['last_wall_s'] = time.monotonic()
        play_button.props(f"icon={'pause' if state['playing'] else 'play_arrow'}")

    def step(delta):
        state['playing'] = False
        play_button.props('icon=play_arrow')
        set_index(state['index'] + delta)

    def shift_window(direction):
        state['playing'] = False
        play_button.props('icon=play_arrow')
        set_window_start(state['window_start_s'] + direction * RAW_WINDOW_SHIFT_S, redraw=False)
        state['window_dirty'] = True

    def command_index_at_sample():
        data = state['data']
        if data is None:
            return None
        time_s = float(data.time_s[state['index']])
        for index, segment in enumerate(data.segments):
            if time_s <= float(segment.get('end_time_s', time_s)):
                return index
        return len(data.segments) - 1

    def jump_command(direction):
        """Focus the raw window on the previous or next planned command."""
        data = state['data']
        if data is None:
            return
        current = command_index_at_sample()
        target = max(0, min(current + direction, len(data.segments) - 1))
        target_time = float(data.segments[target].get('start_time_s', 0.0))
        state['playing'] = False
        play_button.props('icon=play_arrow')
        state['window_start_s'] = target_time
        state['index'] = max(0, min(int(np.searchsorted(data.time_s, target_time, side='left')),
                                    len(data.time_s) - 1))
        draw_all()

    def transition_indices():
        """Map segment starts and virtual events onto their nearest saved sample."""
        data = state['data']
        if data is None:
            return ()
        times = [float(segment.get('start_time_s', 0.0)) for segment in data.segments[1:]]
        times.extend(float(event.get('time_s', 0.0)) for event in data.events)
        return tuple(sorted({
            max(0, min(int(np.searchsorted(data.time_s, time_s, side='left')),
                       len(data.time_s) - 1))
            for time_s in times
        }))
    def jump_transition(direction):
        """Snap to the preceding or following command/action transition."""
        data = state['data']
        if data is None:
            return
        state['playing'] = False
        play_button.props('icon=play_arrow')
        indices = transition_indices()
        if direction < 0:
            targets = [index for index in indices if index < state['index']]
            set_index(targets[-1] if targets else 0)
        else:
            targets = [index for index in indices if index > state['index']]
            set_index(targets[0] if targets else len(data.time_s) - 1)
    def playback_tick():
        active_path = ACTIVE_PREVIEW_PATH
        try:
            active_mtime_ns = active_path.stat().st_mtime_ns
        except OSError:
            active_mtime_ns = None
        if (Path(path_input.value.strip()) == active_path
                and active_mtime_ns is not None
                and active_mtime_ns != state['active_mtime_ns']):
            load_preview()
        data = state['data']
        if data is None:
            return
        if not visible():
            if state['playing']:
                state['playing'] = False
                play_button.props('icon=play_arrow')
            state['window_dirty'] = False
            return
        if state['window_dirty']:
            state['window_dirty'] = False
            draw_all()
        if not state['playing']:
            return
        now = time.monotonic()
        elapsed = max(0.0, now - state['last_wall_s'])
        state['last_wall_s'] = now
        target = min(float(data.time_s[-1]),
                     float(data.time_s[state['index']]) + elapsed * state['speed'])
        set_index(np.searchsorted(data.time_s, target, side='right') - 1)
        if state['index'] >= len(data.time_s) - 1:
            state['playing'] = False
            play_button.props('icon=play_arrow')
    with ui.card().classes('w-full mb-3 rounded-lg shadow-sm'):
        with ui.row().classes('items-center justify-between w-full gap-3 flex-wrap'):
            ui.label('Offline Plan Preview').classes('text-sm font-semibold')
            status = ui.label('Load a dry-run .npz artifact to inspect it safely.') \
                .classes('text-sm text-gray-500')
        with ui.row().classes('items-end gap-2 w-full flex-wrap'):
            path_input = ui.input('Preview file path (.npz)',
                                  value=str(ACTIVE_PREVIEW_PATH)).classes('flex-1 min-w-72')
            path_input.on('keydown.enter', lambda _: load_preview())
            ui.button('Load preview', icon='folder_open', on_click=load_preview) \
                .props('color=primary outline')
            play_button = ui.button(icon='play_arrow', on_click=toggle_play).props('flat round') \
                .tooltip('Play or pause virtual time')
            ui.button(icon='skip_previous', on_click=lambda: step(-1)).props('flat round') \
                .tooltip('Previous planned sample')
            ui.button(icon='skip_next', on_click=lambda: step(1)).props('flat round') \
                .tooltip('Next planned sample')
            ui.label('Transition').classes('text-xs text-gray-500')
            ui.button(icon='keyboard_double_arrow_left', on_click=lambda: jump_transition(-1)).props('flat round').tooltip('Previous command transition')
            ui.button(icon='keyboard_double_arrow_right', on_click=lambda: jump_transition(1)).props('flat round').tooltip('Next command transition')
            ui.label('View').classes('text-xs text-gray-500')
            ui.button(icon='chevron_left', on_click=lambda: shift_window(-1)).props('flat round') \
                .tooltip('Shift raw view 4 seconds left')
            ui.button(icon='chevron_right', on_click=lambda: shift_window(1)).props('flat round') \
                .tooltip('Shift raw view 4 seconds right')
            ui.label('Command').classes('text-xs text-gray-500')
            ui.button(icon='first_page', on_click=lambda: jump_command(-1)).props('flat round') \
                .tooltip('Focus previous planned command')
            ui.button(icon='last_page', on_click=lambda: jump_command(1)).props('flat round') \
                .tooltip('Focus next planned command')
            ui.button(icon='replay', on_click=lambda: set_index(0)).props('flat round') \
                .tooltip('Jump to the first planned sample')
            ui.select({'0.25': '0.25x', '0.5': '0.5x', '1.0': '1x',
                       '2.0': '2x', '4.0': '4x'}, value='1.0',
                      on_change=lambda event: state.__setitem__('speed', float(event.value))) \
                .props('dense outlined')
        sample_label = ui.label('No preview sample loaded.').classes('text-sm font-mono text-gray-500')
        window_label = ui.label('Raw 4 ms view: no preview loaded.').classes('text-xs text-gray-500')
        ui.toggle({
            'joint_position_rad': 'Joint position',
            'joint_velocity_rad_s': 'Joint velocity',
            'joint_acceleration_rad_s2': 'Joint acceleration',
            'joint_jerk_rad_s3': 'Joint jerk',
            'tcp_pose': 'TCP pose',
            'tcp_velocity': 'TCP velocity',
            'tcp_acceleration': 'TCP acceleration',
        }, value='joint_position_rad',
                  on_change=lambda event: (state.__setitem__('metric', event.value), draw_signal())) \
            .props('dense')
        with ui.row().classes('items-center gap-2 w-full mt-2 flex-wrap'):
            ui.label('Signal axes:').classes('text-sm text-gray-500')
            for axis in range(6):
                checkbox = ui.checkbox(str(axis + 1), value=axis == 0, on_change=lambda event, index=axis: set_axis_enabled(index, bool(event.value))).props('dense')
                axis_checkboxes.append(checkbox)
        signal_chart = ui.echart(signal_options()).classes('w-full').style('height:340px;')
        with ui.row().classes('items-center gap-3 w-full flex-wrap'):
            ui.label('Timeline key:').classes('text-xs text-gray-500')
            with ui.row().classes('items-center gap-1'):
                ui.element('span').style('width:12px;height:12px;background:rgba(37,99,235,.18);display:inline-block;')
                ui.label('motion segment A').classes('text-xs')
            with ui.row().classes('items-center gap-1'):
                ui.element('span').style('width:12px;height:12px;background:rgba(5,150,105,.18);display:inline-block;')
                ui.label('motion segment B (alternating)').classes('text-xs')
            with ui.row().classes('items-center gap-1'):
                ui.element('span').style('width:12px;height:12px;background:rgba(217,119,6,.28);display:inline-block;')
                ui.label('delay').classes('text-xs')
            with ui.row().classes('items-center gap-1'):
                ui.element('span').style('width:12px;height:12px;background:rgba(190,24,93,.32);display:inline-block;')
                ui.label('Ruckig corner transition').classes('text-xs')
            with ui.row().classes('items-center gap-1'):
                ui.element('span').style('width:12px;height:12px;background:rgba(124,58,237,.32);display:inline-block;')
                ui.label('exact blend zone').classes('text-xs')
            with ui.row().classes('items-center gap-1'):
                ui.element('span').style('width:20px;border-top:2px solid #0f4c81;display:inline-block;')
                ui.label('move start').classes('text-xs')
            with ui.row().classes('items-center gap-1'):
                ui.element('span').style('width:20px;border-top:2px dashed #7c3aed;display:inline-block;')
                ui.label('setup, I-O, or gripper action').classes('text-xs')
        with ui.row().classes('items-center justify-between w-full mt-2'):
            ui.label('Planned TCP Path').classes('text-sm font-semibold')
            ui.toggle({'xy': 'XY', 'xz': 'XZ', 'yz': 'YZ', 'iso': 'XYZ'}, value='xy', on_change=lambda event: (state.__setitem__('path_mode', event.value), draw_path())).props('dense')
        path_chart = ui.echart(path_options()).style('width:min(100%, 620px);height:min(620px, calc(100vw - 48px));')
        sample_columns = [
            {'name': 'axis', 'label': 'Axis', 'field': 'axis'},
            {'name': 'joint_position', 'label': 'Joint pos [rad]', 'field': 'joint_position'},
            {'name': 'joint_velocity', 'label': 'Joint vel [rad/s]', 'field': 'joint_velocity'},
            {'name': 'joint_acceleration', 'label': 'Joint acc [rad/s^2]', 'field': 'joint_acceleration'},
            {'name': 'joint_jerk', 'label': 'Joint jerk [rad/s^3]', 'field': 'joint_jerk'},
            {'name': 'tcp_pose', 'label': 'TCP pose [m/rad]', 'field': 'tcp_pose'},
            {'name': 'tcp_velocity', 'label': 'TCP vel [m/s/rad/s]', 'field': 'tcp_velocity'},
            {'name': 'tcp_acceleration', 'label': 'TCP acc [m/s^2/rad/s^2]', 'field': 'tcp_acceleration'},
        ]
        sample_table = ui.table(columns=[{**c, 'label': unit_text(c['label'])} for c in sample_columns],
                                rows=[]).classes('w-full').props('dense flat')
        ui.label('Program Diagnostics').classes('text-sm font-semibold mt-3')
        diagnostic_columns = [
            {'name': 'sequence', 'label': '#', 'field': 'sequence'},
            {'name': 'command', 'label': 'Command', 'field': 'command'},
            {'name': 'profile', 'label': 'Profile', 'field': 'profile'},
            {'name': 'duration', 'label': 'Duration [s]', 'field': 'duration'},
            {'name': 'samples', 'label': 'Samples', 'field': 'samples'},
            {'name': 'end_joints', 'label': 'End joints [deg]', 'field': 'end_joints'},
            {'name': 'end_tcp', 'label': 'End TCP XYZ [mm]', 'field': 'end_tcp'},
            {'name': 'peak_velocity', 'label': 'Peak vel [rad/s]', 'field': 'peak_velocity'},
            {'name': 'peak_acceleration', 'label': 'Peak acc [rad/s^2]', 'field': 'peak_acceleration'},
            {'name': 'peak_jerk', 'label': 'Peak jerk [rad/s^3]', 'field': 'peak_jerk'},
            {'name': 'sigma_min', 'label': 'sigma_min', 'field': 'sigma_min'},
            {'name': 'condition', 'label': 'Condition', 'field': 'condition'},
            {'name': 'status', 'label': 'Diagnostics', 'field': 'status'},
        ]
        diagnostic_table = ui.table(columns=[{**c, 'label': unit_text(c['label'])} for c in diagnostic_columns],
                                    rows=[]).classes('w-full').props('dense flat wrap-cells')
    # Load the one managed active preview when the page is opened. A later run of
    # the offline planner is picked up by playback_tick via its changed mtime.
    if ACTIVE_PREVIEW_PATH.exists():
        load_preview()
    ui.timer(1.0 / 30.0, playback_tick)

    def set_units():
        """Redraw in the current angle unit (the dashboard's Angles toggle)."""
        sample_table.columns = [{**c, 'label': unit_text(c['label'])} for c in sample_columns]
        diagnostic_table.columns = [{**c, 'label': unit_text(c['label'])} for c in diagnostic_columns]
        update_diagnostics()
        draw_all()

    return set_units
