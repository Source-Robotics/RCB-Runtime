// Block SHAPES only. What a block DOES is blockly/codegen.py (TODO/blockly_plan.md D1).
'use strict';

const RCB_PINS = [['1', '1'], ['2', '2'], ['3', '3']];

Blockly.common.defineBlocksWithJsonArray([
  {
    type: 'rcb_gripper_open',
    message0: 'open the gripper',
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_gripper',
    tooltip: 'Open the jaws; the next block starts when they have stopped.',
  },
  {
    type: 'rcb_gripper_close',
    message0: 'close the gripper',
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_gripper',
    tooltip: 'Close the jaws; the next block starts when they have stopped — on a part or fully closed.',
  },
  {
    type: 'rcb_gripper_move',
    message0: 'move the gripper to %1 %% closed',
    args0: [{type: 'field_number', name: 'PERCENT', value: 50, min: 0, max: 100, precision: 1}],
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_gripper',
    tooltip: '0 % = fully open, 100 % = fully closed (the gripper\'s own scale, not millimetres).',
  },
  {
    type: 'rcb_gripper_holding',
    message0: 'the gripper holds a part',
    output: 'Boolean',
    style: 'rcb_gripper',
    tooltip: 'True if the last gripper move stopped on something.',
  },
  {
    type: 'rcb_set_output',
    message0: 'set output %1 %2',
    args0: [
      {type: 'field_dropdown', name: 'PIN', options: RCB_PINS},
      {type: 'field_dropdown', name: 'STATE', options: [['on', 'ON'], ['off', 'OFF']]},
    ],
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_io',
    tooltip: 'Switch an isolated output of the control box.',
  },
  {
    type: 'rcb_input',
    message0: 'input %1 is on',
    args0: [{type: 'field_dropdown', name: 'PIN', options: RCB_PINS}],
    output: 'Boolean',
    style: 'rcb_io',
    tooltip: 'True while this isolated input of the control box is on.',
  },
  {
    type: 'rcb_wait_input',
    message0: 'wait until input %1 is %2, at most %3 s',
    args0: [
      {type: 'field_dropdown', name: 'PIN', options: RCB_PINS},
      {type: 'field_dropdown', name: 'STATE', options: [['on', 'ON'], ['off', 'OFF']]},
      {type: 'field_number', name: 'TIMEOUT', value: 10, min: 0.1, max: 3600, precision: 0.1},
    ],
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_io',
    tooltip: 'Wait for an input. If it does not come in time, the program stops with an error.',
  },
]);
