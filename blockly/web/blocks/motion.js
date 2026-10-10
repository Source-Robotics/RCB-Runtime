// Block SHAPES only. What a block DOES is blockly/codegen.py (TODO/blockly_plan.md D1).
// Every move finishes before the next block starts (TODO/blockly_plan.md §7).
'use strict';

Blockly.common.defineBlocksWithJsonArray([
  {
    type: 'rcb_move_joints',
    message0: 'move to %1 with joints, speed %2 %%',
    args0: [
      {type: 'input_value', name: 'TARGET', check: ['Point', 'Joints', 'Pose']},
      {type: 'field_number', name: 'SPEED', value: 20, min: 1, max: 100, precision: 1},
    ],
    inputsInline: true,
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_motion',
    tooltip: 'Move every joint to the target; the TCP path is not a straight line. ' +
             'Speed is a percent of the speed the robot allows for programs.',
  },
  {
    type: 'rcb_move_linear',
    message0: 'move to %1 in a straight line, speed %2 %%',
    args0: [
      {type: 'input_value', name: 'TARGET', check: ['Point', 'Pose']},
      {type: 'field_number', name: 'SPEED', value: 20, min: 1, max: 100, precision: 1},
    ],
    inputsInline: true,
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_motion',
    tooltip: 'Move the tool in a straight line to the target. The robot caps straight-line moves at 20 % speed. ' +
             'Joint angles alone cannot be a straight-line target: use a point or a pose.',
  },
  {
    type: 'rcb_move_relative',
    message0: 'move by x %1 y %2 z %3 mm in the %4 frame, speed %5 %%',
    args0: [
      {type: 'field_number', name: 'DX', value: 0, min: -2000, max: 2000, precision: 0.1},
      {type: 'field_number', name: 'DY', value: 0, min: -2000, max: 2000, precision: 0.1},
      {type: 'field_number', name: 'DZ', value: 10, min: -2000, max: 2000, precision: 0.1},
      {type: 'field_dropdown', name: 'FRAME', options: [['world', 'world'], ['tool', 'tool']]},
      {type: 'field_number', name: 'SPEED', value: 20, min: 1, max: 100, precision: 1},
    ],
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_motion',
    tooltip: 'Move the tool in a straight line by this much, along the world axes or the tool\'s own axes.',
  },
  {
    type: 'rcb_turn_joint',
    message0: 'turn joint %1 by %2 °, speed %3 %%',
    args0: [
      {type: 'field_dropdown', name: 'JOINT', options: [1, 2, 3, 4, 5, 6].map((i) => ['J' + i, String(i)])},
      {type: 'field_number', name: 'DEG', value: 5, min: -720, max: 720, precision: 0.01},
      {type: 'field_number', name: 'SPEED', value: 20, min: 1, max: 100, precision: 1},
    ],
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_motion',
    tooltip: 'Turn one joint by this many degrees from where it is now.',
  },
  {
    type: 'rcb_wait',
    message0: 'wait %1 s',
    args0: [{type: 'field_number', name: 'SECONDS', value: 1, min: 0, max: 86400, precision: 0.1}],
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_motion',
    tooltip: 'Do nothing for this many seconds (a pause stops the clock).',
  },
]);
