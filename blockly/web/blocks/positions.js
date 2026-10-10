// Block SHAPES only. What a block DOES is blockly/codegen.py (TODO/blockly_plan.md D1).
//
// A move's target is a NAMED POINT from the program's points table, or VALUES IN THE BLOCK (joints / pose) — both,
// picked per block (decision D2). Joint values fit a joint move only (no FK on the client); a pose fits both moves.
'use strict';

// The points table the page shows; the page keeps it current with RCBBlocks / editor.setPoints().
const RCB_POINTS = {names: []};

// A dropdown of the program's points that NEVER changes a block's point by itself: Blockly's dropdown refuses a
// value that is not among its options and falls back to the first one — a move would silently go somewhere else
// after its point was deleted or renamed. This one keeps the name, shows "(missing)", and the Pi refuses to run it.
// getOptions is overridden (not a menu generator): Blockly calls a generator without `this`.
class RcbFieldPoint extends Blockly.FieldDropdown {
  constructor(value) {
    super(() => [['(no points yet)', '']]);
    if (value !== undefined) this.setValue(value);
  }

  static fromJson(options) {
    return new RcbFieldPoint(options.value);
  }

  getOptions() {
    const current = this.getValue();
    const options = RCB_POINTS.names.map((n) => [n, n]);
    if (current && !RCB_POINTS.names.includes(current)) options.push([current + ' (missing)', current]);
    if (!options.length) options.push(['(no points yet)', '']);
    return options;
  }

  doClassValidation_(value) {
    return typeof value === 'string' ? value : null;
  }
}
Blockly.fieldRegistry.register('field_rcb_point', RcbFieldPoint);

// "Use the current position" in the right-click menu of a joints / pose block: fills it from the arm.
Blockly.Extensions.registerMixin('rcb_use_current', {
  customContextMenu(options) {
    const block = this;
    options.unshift({
      text: 'Use the current position',
      enabled: !block.workspace.isReadOnly() && !block.isInFlyout,
      callback: () => RCBBlocks.fillFromArm(block),
    });
  },
});

Blockly.common.defineBlocksWithJsonArray([
  {
    type: 'rcb_point',
    message0: 'point %1',
    args0: [{type: 'field_rcb_point', name: 'NAME'}],
    output: 'Point',
    style: 'rcb_position',
    tooltip: 'A point taught from the arm (the Points list beside the program). Re-teach it once and every block ' +
             'that uses it follows.',
  },
  {
    type: 'rcb_joints',
    message0: 'joints J1 %1 J2 %2 J3 %3 J4 %4 J5 %5 J6 %6 °',
    args0: [1, 2, 3, 4, 5, 6].map((i) => ({type: 'field_number', name: 'J' + i, value: 0, precision: 0.01})),
    inputsInline: true,
    output: 'Joints',
    style: 'rcb_position',
    extensions: ['rcb_use_current'],
    tooltip: 'Joint angles in degrees, J1 (base) to J6 (wrist). Right-click: use the current position.',
  },
  {
    type: 'rcb_pose',
    message0: 'pose x %1 y %2 z %3 mm  rx %4 ry %5 rz %6 °',
    args0: [
      ...['X', 'Y', 'Z'].map((n) => ({type: 'field_number', name: n, value: 0, min: -5000, max: 5000, precision: 0.1})),
      ...['RX', 'RY', 'RZ'].map((n) => ({type: 'field_number', name: n, value: 0, min: -720, max: 720, precision: 0.01})),
    ],
    inputsInline: true,
    output: 'Pose',
    style: 'rcb_position',
    extensions: ['rcb_use_current'],
    tooltip: 'A tool pose in the world frame: position in mm, rotation (roll, pitch, yaw) in degrees. ' +
             'Right-click: use the current position.',
  },
  {
    type: 'rcb_here',
    message0: 'current position',
    output: 'Point',
    style: 'rcb_position',
    tooltip: 'Where the arm is when this block runs.',
  },
  {
    type: 'rcb_shift',
    message0: '%1 shifted by x %2 y %3 z %4 mm',
    args0: [
      {type: 'input_value', name: 'TARGET', check: ['Point', 'Pose']},
      ...['DX', 'DY', 'DZ'].map((n) => ({type: 'field_number', name: n, value: 0, min: -5000, max: 5000, precision: 0.1})),
    ],
    inputsInline: true,
    output: 'Pose',
    style: 'rcb_position',
    tooltip: 'The same tool pose moved along the world axes, e.g. "above the pick point": z 50.',
  },
]);
