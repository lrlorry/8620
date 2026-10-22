# A1 LaTeX report template

`u8076655_A1_report.tex` is a report skeleton for COMP8620 Assignment 1.
It follows the question-by-question structure of the existing SML A1 report,
while separating derivations, pseudocode, implementation notes, experiments,
and the required generative-AI disclosure.

## Compile

From this directory, run:

```bash
latexmk -pdf u8076655_A1_report.tex
```

The generated report is `u8076655_A1_report.pdf`. To remove LaTeX auxiliary
files without deleting the PDF, run:

```bash
latexmk -c u8076655_A1_report.tex
```

## Before writing

1. Confirm the student details near the top of `u8076655_A1_report.tex`.
2. Replace every visible `[Replace this placeholder: ...]` instruction.
3. Remove algorithm blocks that do not apply to a No answer in A1(a) or A1(b).
4. Make the B2 pseudocode agree exactly with the submitted B3 implementation.
5. Replace the B4 table rows with the actual experimental conditions and values.
6. Include every generative-AI prompt verbatim in Appendix A.
7. Remove unused placeholder references and optional appendix material.

## Submission reminder

The report PDF and source-code archive are separate Canvas submissions. The
code archive must also contain the environments and test cases used to produce
the B4 results; they are not embedded in this LaTeX report template.

This is a student-created template, not an official course template. If Canvas
or Ed publishes newer formatting or submission instructions, those instructions
take precedence.
