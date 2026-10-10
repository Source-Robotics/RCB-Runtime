// The block editor as a component any page can host:
//
//   const editor = await RCBBlocks.mount(div, {api: 'api'});   // resolves once `div` has a size
//   editor.load(state) / editor.save()            the program as JSON (Blockly.serialization.workspaces)
//   editor.setReadOnly(true)                      while a program runs (block ids must not change under it)
//   editor.highlight(id) / editor.markError(id, message) / editor.clearErrors()
//   editor.setPoints(names)                       the program's point names, for the point dropdowns
//   await editor.generate(name, points)           -> {ok, python} or {ok: false, block, message}, from the Pi
//
// It only edits. The Pi writes the Python (blockly/codegen.py) and, later, runs it — TODO/blockly_plan.md §4.
'use strict';

const RCBBlocks = (() => {
  // Relative to the page. Blockly's default is static.blockly.com/media — the robot has no internet.
  const MEDIA = 'vendor/blockly-13.3.0/media/';

  const THEME = Blockly.Theme.defineTheme('rcb', {
    base: Blockly.Themes.Classic,
    blockStyles: {
      rcb_program: {colourPrimary: '#b8742a', hat: 'cap'},
      rcb_motion: {colourPrimary: '#3f6fc9'},
      rcb_position: {colourPrimary: '#5c7ea8'},
      rcb_gripper: {colourPrimary: '#2f8f83'},
      rcb_io: {colourPrimary: '#8a5a9e'},
    },
    categoryStyles: {
      rcb_program: {colour: '#b8742a'},
      rcb_motion: {colour: '#3f6fc9'},
      rcb_position: {colour: '#5c7ea8'},
      rcb_gripper: {colour: '#2f8f83'},
      rcb_io: {colour: '#8a5a9e'},
    },
  });

  // The API base of the editor mounted last ("use the current position" asks it for the arm's position).
  let apiBase = 'api';

  // The program's point names, for every point dropdown (blocks/positions.js). Dropdowns re-read them when opened;
  // existing blocks are re-rendered so a "(missing)" label appears or disappears at once.
  function setPoints(ws, names) {
    RCB_POINTS.names = names.slice().sort();
    if (!ws) return;
    for (const block of ws.getAllBlocks(false)) {
      if (block.type !== 'rcb_point') continue;
      const field = block.getField('NAME');
      field.doValueUpdate_(field.getValue());     // setValue() of the same value is skipped; this re-picks the label
      field.forceRerender();
    }
  }

  async function fetchPosition() {
    const resp = await fetch(apiBase + '/robot/position');
    const body = await resp.json();
    if (!body.ok) throw new Error(body.message);
    return body;
  }

  // Fill a joints / pose block with where the arm is — one undo step.
  async function fillFromArm(block) {
    let pos;
    try {
      pos = await fetchPosition();
    } catch (e) {
      block.setWarningText('Could not read the arm\'s position: ' + e.message);
      return false;
    }
    const round = (v, digits) => Number(v.toFixed(digits));
    Blockly.Events.setGroup(true);
    try {
      if (block.type === 'rcb_joints') {
        pos.joints_deg.forEach((v, i) => block.setFieldValue(round(v, 2), 'J' + (i + 1)));
      } else if (block.type === 'rcb_pose') {
        ['X', 'Y', 'Z'].forEach((n, i) => block.setFieldValue(round(pos.pose[i] * 1000, 1), n));
        ['RX', 'RY', 'RZ'].forEach((n, i) => block.setFieldValue(round(pos.pose[i + 3] * 180 / Math.PI, 2), n));
      }
    } finally {
      Blockly.Events.setGroup(false);
    }
    block.setWarningText(null);
    return true;
  }

  // Inject only once the container has a size. In a dashboard the editor's iframe usually loads while its tab is
  // hidden (0 x 0), and a workspace laid out at 0 x 0 places its toolbox and scrollbars wrongly.
  function mount(div, options = {}) {
    return new Promise((resolve) => {
      const hasSize = () => div.clientWidth > 0 && div.clientHeight > 0;
      if (hasSize()) return resolve(inject(div, options));
      const watch = new ResizeObserver(() => {
        if (hasSize()) {
          watch.disconnect();
          resolve(inject(div, options));
        }
      });
      watch.observe(div);
    });
  }

  function inject(div, options) {
    const ws = Blockly.inject(div, {
      toolbox: options.toolbox || RCB_TOOLBOX,
      theme: THEME,
      media: options.media || MEDIA,
      trashcan: true,
      zoom: {controls: true, wheel: true, startScale: 0.9, maxScale: 2, minScale: 0.4},
      move: {scrollbars: true, drag: true, wheel: false},
      grid: {spacing: 25, length: 3, colour: '#e4e4e4', snap: true},
    });
    // Blockly follows the WINDOW's size; a tab shown or hidden changes only the container.
    new ResizeObserver(() => {
      if (div.clientWidth > 0) Blockly.svgResize(ws);
    }).observe(div);
    const api = options.api || 'api';
    apiBase = api;

    return {
      workspace: ws,
      setPoints(names) {
        setPoints(ws, names);
      },
      load(state) {
        Blockly.serialization.workspaces.load(state, ws);
      },
      save() {
        return Blockly.serialization.workspaces.save(ws);
      },
      setReadOnly(on) {
        ws.setIsReadOnly(!!on);
      },
      isReadOnly() {
        return ws.isReadOnly();
      },
      highlight(id) {
        ws.highlightBlock(id || null);
      },
      markError(id, message) {
        const block = id ? ws.getBlockById(id) : null;
        if (!block) return false;
        block.setWarningText(message);
        Blockly.common.setSelected(block);
        ws.centerOnBlock(id);
        return true;
      },
      clearErrors() {
        for (const block of ws.getAllBlocks(false)) block.setWarningText(null);
      },
      onChange(fn) {
        ws.addChangeListener((event) => {
          if (!event.isUiEvent) fn(event);
        });
      },
      async generate(name, points) {
        const resp = await fetch(api + '/codegen', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({workspace: this.save(), name, points: points || {}}),
        });
        if (!resp.ok) return {ok: false, block: null, message: `the robot answered HTTP ${resp.status}`};
        return resp.json();
      },
    };
  }

  return {mount, MEDIA, fillFromArm, fetchPosition};
})();
