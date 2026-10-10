// The block palette. Every block type here must have a function in blockly/codegen.py
// (dummy_test_scripts/test_block_codegen.py fails otherwise).
'use strict';

const RCB_TOOLBOX = (() => {
  const number = (n) => ({shadow: {type: 'math_number', fields: {NUM: n}}});
  return {
    kind: 'categoryToolbox',
    contents: [
      {kind: 'category', name: 'Program', categorystyle: 'rcb_program', contents: [
        {kind: 'block', type: 'rcb_start'},
        {kind: 'block', type: 'rcb_log', inputs: {TEXT: {shadow: {type: 'text', fields: {TEXT: 'hello'}}}}},
        {kind: 'block', type: 'rcb_halt'},
      ]},
      {kind: 'category', name: 'Motion', categorystyle: 'rcb_motion', contents: [
        {kind: 'block', type: 'rcb_move_joints', inputs: {TARGET: {block: {type: 'rcb_point'}}}},
        {kind: 'block', type: 'rcb_move_joints', inputs: {TARGET: {block: {type: 'rcb_joints'}}}},
        {kind: 'block', type: 'rcb_move_linear', inputs: {TARGET: {block: {type: 'rcb_point'}}}},
        {kind: 'block', type: 'rcb_move_relative'},
        {kind: 'block', type: 'rcb_turn_joint'},
        {kind: 'block', type: 'rcb_wait'},
      ]},
      {kind: 'category', name: 'Positions', categorystyle: 'rcb_position', contents: [
        {kind: 'block', type: 'rcb_point'},
        {kind: 'block', type: 'rcb_joints'},
        {kind: 'block', type: 'rcb_pose'},
        {kind: 'block', type: 'rcb_shift', inputs: {TARGET: {block: {type: 'rcb_point'}}}},
        {kind: 'block', type: 'rcb_here'},
      ]},
      {kind: 'category', name: 'Gripper', categorystyle: 'rcb_gripper', contents: [
        {kind: 'block', type: 'rcb_gripper_open'},
        {kind: 'block', type: 'rcb_gripper_close'},
        {kind: 'block', type: 'rcb_gripper_move'},
        {kind: 'block', type: 'rcb_gripper_holding'},
      ]},
      {kind: 'category', name: 'I/O', categorystyle: 'rcb_io', contents: [
        {kind: 'block', type: 'rcb_set_output'},
        {kind: 'block', type: 'rcb_input'},
        {kind: 'block', type: 'rcb_wait_input'},
      ]},
      {kind: 'sep'},
      {kind: 'category', name: 'Logic', categorystyle: 'logic_category', contents: [
        {kind: 'block', type: 'controls_if'},
        {kind: 'block', type: 'controls_if', extraState: {hasElse: true}},
        {kind: 'block', type: 'logic_compare'},
        {kind: 'block', type: 'logic_operation'},
        {kind: 'block', type: 'logic_negate'},
        {kind: 'block', type: 'logic_boolean'},
      ]},
      {kind: 'category', name: 'Loops', categorystyle: 'loop_category', contents: [
        {kind: 'block', type: 'controls_repeat_ext', inputs: {TIMES: number(3)}},
        {kind: 'block', type: 'controls_whileUntil'},
        {kind: 'block', type: 'controls_for', inputs: {FROM: number(1), TO: number(10), BY: number(1)}},
        {kind: 'block', type: 'controls_flow_statements'},
      ]},
      {kind: 'category', name: 'Math', categorystyle: 'math_category', contents: [
        {kind: 'block', type: 'math_number'},
        {kind: 'block', type: 'math_arithmetic', inputs: {A: number(1), B: number(1)}},
        {kind: 'block', type: 'text'},
      ]},
      {kind: 'category', name: 'Variables', categorystyle: 'variable_category', custom: 'VARIABLE'},
    ],
  };
})();
