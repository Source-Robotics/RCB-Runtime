// Block SHAPES only. What a block DOES is blockly/codegen.py (TODO/blockly_plan.md D1): a block added here needs
// its function there, or the Pi refuses to run it (dummy_test_scripts/test_block_codegen.py checks the pairing).
'use strict';

Blockly.common.defineBlocksWithJsonArray([
  {
    type: 'rcb_start',
    message0: 'when the program starts',
    nextStatement: null,
    style: 'rcb_program',
    tooltip: 'The program: only the blocks attached under this one run.',
  },
  {
    type: 'rcb_halt',
    message0: 'end the program here',
    previousStatement: null,
    style: 'rcb_program',
    tooltip: 'The program ends normally; the arm stays where it is.',
  },
  {
    type: 'rcb_log',
    message0: 'log %1',
    args0: [{type: 'input_value', name: 'TEXT'}],
    previousStatement: null,
    nextStatement: null,
    style: 'rcb_program',
    tooltip: 'Show a message in the log beside the program.',
  },
]);
