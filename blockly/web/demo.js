// The demo program, in the format Blockly.serialization.workspaces.save() writes (Blockly gives the ids on load).
// J5 only, by 5 degrees — the habit of every first test on the arm.
'use strict';

const RCB_DEMO = (() => {
  const joints = (j5) => ({type: 'rcb_joints', fields: {J1: 90, J2: -105, J3: 165, J4: 0, J5: j5, J6: 180}});
  const move = (j5, next) => ({
    type: 'rcb_move_joints', fields: {SPEED: 20}, inputs: {TARGET: {block: joints(j5)}},
    ...(next ? {next: {block: next}} : {}),
  });
  const wait = (s, next) => ({type: 'rcb_wait', fields: {SECONDS: s}, ...(next ? {next: {block: next}} : {})});
  const repeat = {
    type: 'controls_repeat_ext',
    inputs: {
      TIMES: {shadow: {type: 'math_number', fields: {NUM: 2}}},
      DO: {block: move(-25, wait(0.5, move(-30)))},
    },
  };
  return {blocks: {languageVersion: 0, blocks: [
    {type: 'rcb_start', x: 40, y: 40, next: {block: move(-30, repeat)}},
  ]}};
})();
