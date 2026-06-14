#!/usr/bin/env python3
"""
Generate BARQ Report 2 (funder progress report) as a LaTeX-style PDF via reportlab.

Content is derived from the project logs (docs/01_STATUS, 03_CHANGELOG, 05_RESEARCH_LOG,
02_DECISIONS) as of 2026-06-14. Rerun to regenerate; copy+edit for Report 3.

  pip install reportlab     # host has 4.5.1
  python3 generate_report2.py
Outputs BARQ_Report_2.pdf next to this script.
"""

import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import cm, mm
from reportlab.platypus import (BaseDocTemplate, Frame, Image, PageTemplate,
                                Paragraph, Spacer, Table, TableStyle, KeepTogether)

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, '..', 'research', '2026-06-13-torque-budget.png')
OUT = os.path.join(HERE, 'BARQ_Report_2.pdf')
SERIF, SERIF_B, SERIF_I = 'Times-Roman', 'Times-Bold', 'Times-Italic'
INK = colors.HexColor('#1a1a1a')
RULE = colors.HexColor('#888888')
ACCENT = colors.HexColor('#333333')

styles = getSampleStyleSheet()
body = ParagraphStyle('body', parent=styles['Normal'], fontName=SERIF, fontSize=10.5,
                      leading=15, alignment=TA_JUSTIFY, textColor=INK, spaceAfter=6)
h1 = ParagraphStyle('h1', fontName=SERIF_B, fontSize=14, leading=18, textColor=INK,
                    spaceBefore=14, spaceAfter=6)
h2 = ParagraphStyle('h2', fontName=SERIF_B, fontSize=11.5, leading=15, textColor=ACCENT,
                    spaceBefore=9, spaceAfter=3)
cap = ParagraphStyle('cap', parent=body, fontName=SERIF_I, fontSize=9, leading=12,
                     alignment=TA_CENTER, textColor=ACCENT, spaceBefore=4)
cell = ParagraphStyle('cell', parent=body, fontSize=9.5, leading=12, alignment=TA_JUSTIFY,
                      spaceAfter=0)
cellb = ParagraphStyle('cellb', parent=cell, fontName=SERIF_B)


def section(n, title):
    return Paragraph(f'{n}&nbsp;&nbsp;{title}', h1)


def sub(title):
    return Paragraph(title, h2)


def p(txt):
    return Paragraph(txt, body)


def metric_table(rows, col_widths, header=True):
    data = [[Paragraph(c, cellb if (header and i == 0) else cell) for c in r]
            for i, r in enumerate(rows)]
    t = Table(data, colWidths=col_widths, hAlign='CENTER')
    ts = [('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
          ('LINEBELOW', (0, 0), (-1, 0), 0.75, INK),
          ('LINEABOVE', (0, 0), (-1, 0), 0.75, INK),
          ('LINEBELOW', (0, -1), (-1, -1), 0.75, INK),
          ('TOPPADDING', (0, 0), (-1, -1), 3),
          ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
          ('LEFTPADDING', (0, 0), (-1, -1), 5),
          ('RIGHTPADDING', (0, 0), (-1, -1), 5)]
    if header:
        ts.append(('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#f0f0f0')))
    t.setStyle(TableStyle(ts))
    return t


def header_footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.5)
    canvas.line(2 * cm, A4[1] - 1.5 * cm, A4[0] - 2 * cm, A4[1] - 1.5 * cm)
    canvas.setFont(SERIF_I, 8)
    canvas.setFillColor(ACCENT)
    canvas.drawString(2 * cm, A4[1] - 1.35 * cm, 'BARQ — Report 2')
    canvas.drawRightString(A4[0] - 2 * cm, A4[1] - 1.35 * cm, 'Status and progress')
    canvas.line(2 * cm, 1.4 * cm, A4[0] - 2 * cm, 1.4 * cm)
    canvas.drawString(2 * cm, 1.0 * cm, 'Confidential — prepared for funder review')
    canvas.drawRightString(A4[0] - 2 * cm, 1.0 * cm, f'Page {doc.page}')
    canvas.restoreState()


def build():
    doc = BaseDocTemplate(OUT, pagesize=A4, leftMargin=2 * cm, rightMargin=2 * cm,
                          topMargin=2.1 * cm, bottomMargin=1.8 * cm, title='BARQ — Report 2',
                          author='Aryaman Gupta, Krish Agarwal')
    frame = Frame(doc.leftMargin, doc.bottomMargin,
                  A4[0] - doc.leftMargin - doc.rightMargin,
                  A4[1] - doc.topMargin - doc.bottomMargin, id='main')
    doc.addPageTemplates([PageTemplate(id='all', frames=[frame], onPage=header_footer)])
    s = []

    # ── title block ──
    s.append(Spacer(1, 1.6 * cm))
    s.append(Paragraph('BARQ', ParagraphStyle('title', fontName=SERIF_B, fontSize=42,
             leading=46, alignment=TA_CENTER, textColor=INK)))
    s.append(Spacer(1, 3 * mm))
    s.append(Paragraph('Report 2', ParagraphStyle('sub', fontName=SERIF, fontSize=18,
             leading=22, alignment=TA_CENTER, textColor=ACCENT)))
    s.append(Spacer(1, 2 * mm))
    s.append(Paragraph('Status and Progress', ParagraphStyle('sub2', fontName=SERIF_I,
             fontSize=13, leading=18, alignment=TA_CENTER, textColor=ACCENT)))
    s.append(Spacer(1, 8 * mm))
    s.append(Paragraph('Aryaman Gupta&nbsp;&nbsp;&middot;&nbsp;&nbsp;Krish Agarwal',
             ParagraphStyle('auth', fontName=SERIF, fontSize=12.5, leading=16,
                            alignment=TA_CENTER, textColor=INK)))
    s.append(Spacer(1, 2 * mm))
    s.append(Paragraph('Autonomous Quadruped Robotics Programme &nbsp;&middot;&nbsp; 14 June 2026',
             ParagraphStyle('date', fontName=SERIF, fontSize=10.5, leading=14,
                            alignment=TA_CENTER, textColor=ACCENT)))
    s.append(Spacer(1, 6 * mm))
    s.append(Table([['']], colWidths=[6 * cm],
                   style=TableStyle([('LINEBELOW', (0, 0), (-1, -1), 1, INK)]), hAlign='CENTER'))
    s.append(Spacer(1, 8 * mm))

    # ── abstract ──
    s.append(Paragraph('Abstract', ParagraphStyle('abh', fontName=SERIF_B, fontSize=11,
             alignment=TA_CENTER, textColor=INK, spaceAfter=4)))
    s.append(Paragraph(
        'BARQ is a twelve-degree-of-freedom quadruped robot. This report covers the development '
        'period since Report 1, during which the complete control, perception, autonomy and '
        'embedded-firmware stack was brought to maturity in high-fidelity physics simulation and '
        'validated end-to-end against an emulated flight controller. Every milestone planned for '
        'the pre-hardware phase is complete: the robot walks under velocity command, maps and '
        'navigates an unknown obstacle course autonomously, estimates its own motion without '
        'external reference, and exercises the exact embedded protocol it will use on metal — all '
        'with zero physical hardware attached. The simulator has been calibrated to the measured '
        'mass and the published actuator specification, making it a faithful digital twin rather '
        'than an idealisation. In parallel, a complete, self-contained execution plan from the '
        'current state to a fully functioning physical robot has been authored, substantially '
        'de-risking the build. Components are now arriving for physical assembly.',
        ParagraphStyle('abs', parent=body, fontSize=10, leading=14, leftIndent=8 * mm,
                       rightIndent=8 * mm, spaceAfter=2)))
    s.append(Spacer(1, 4 * mm))

    # ── 1 executive summary ──
    s.append(section('1', 'Executive summary'))
    s.append(p(
        'The programme strategy is to front-load engineering into simulation so that scarce and '
        'costly hardware iterations are minimised. That strategy has paid off in this period. The '
        'full software pipeline &mdash; velocity command, gait generation, exact inverse '
        'kinematics, motor control, and physics &mdash; runs in the Gazebo simulator with the '
        'robot translating forward, level and straight, and standing-height predicted by our '
        'model to within 0.2&nbsp;mm of the simulated physics. On top of this foundation we have '
        'demonstrated autonomous navigation through a populated 16&nbsp;m obstacle course, an '
        'onboard motion estimator that needs no external tracking, and a firmware-and-interface '
        'layer proven against an emulated controller to the point where commissioning the real '
        'electronics reduces to a single configuration change.'))
    s.append(p(
        'Crucially, the simulation is no longer idealised: it now carries the team&rsquo;s '
        'measured link masses and the manufacturer&rsquo;s actuator torque and speed limits, and '
        'the actuator response has been matched to the servo specification. Results obtained in '
        'simulation are therefore quantitatively meaningful &mdash; including the actuator load '
        'analysis presented in Section&nbsp;2.7, which confirms the chosen servos carry the '
        'walking gait with a comfortable continuous safety margin.'))

    # ── 2 status and progress ──
    s.append(section('2', 'Status and progress'))

    s.append(sub('2.1&nbsp;&nbsp;Simulation and control foundation'))
    s.append(p(
        'The control stack was brought up in stages, each gated by a measurable fidelity check: '
        'kinematic visualisation, simulated motor control, analytical inverse kinematics, trot-gait '
        'generation, and finally full rigid-body physics with gravity and ground contact. A key '
        'correction in this period replaced an idealised leg model with one derived exactly from '
        'the robot&rsquo;s mechanical definition; this was identified through an adversarial '
        'multi-agent design review and eliminated a 3.4&nbsp;cm foot-placement error that had '
        'reversed the walking direction. Following the fix, the model predicts the robot&rsquo;s '
        'settled standing height to within 0.2&nbsp;mm of the physics &mdash; a fifty-fold '
        'improvement and our headline indicator that the simulation faithfully represents the '
        'machine.'))

    s.append(sub('2.2&nbsp;&nbsp;Physical-fidelity calibration'))
    s.append(p(
        'The simulated robot was updated with the team&rsquo;s measured component masses '
        '(2.448&nbsp;kg total, including the battery) and the published Waveshare ST3215 actuator '
        'limits: 2.94&nbsp;N&middot;m peak torque (30&nbsp;kg&middot;cm) and 4.71&nbsp;rad/s '
        'no-load speed. The servo&rsquo;s internal control stiffness was matched to specification '
        'and the resulting joint-tracking error verified at 17.8&nbsp;milliradians RMS during a '
        'walk. These steps convert the simulator from a qualitative animation into a quantitative '
        'digital twin whose force and torque outputs can be trusted (Section&nbsp;2.7).'))

    s.append(sub('2.3&nbsp;&nbsp;Locomotion performance'))
    s.append(p(
        'Under velocity command the robot walks forward, straight and level. It currently realises '
        'approximately 60% of the commanded forward speed in open-loop walking; the cause of the '
        'shortfall was diagnosed rigorously (a swing-foot ground-clearance limit, isolated by a '
        'controlled friction sweep) and is fully recoverable with the closed-loop feedback and '
        'learned-policy work planned for later phases. A deliberate stance trim shifts load toward '
        'the front feet for fore-aft stability. The gait was also shown to compose linear and '
        'turning commands correctly, tracing a clean closed circle of the expected radius.'))

    s.append(sub('2.4&nbsp;&nbsp;Perception and autonomous navigation'))
    s.append(p(
        'A simulated 2-D laser scanner (matched to the specification of the intended sensor) feeds '
        'an industry-standard mapping and navigation stack. The robot built a map of an unknown '
        'environment and then completed a 16&nbsp;metre autonomous mission through a course '
        'comprising a one-metre doorway, a three-pillar slalom and a scattered-obstacle field, '
        'finishing 0.157&nbsp;m from the commanded waypoint and recovering from mid-course stalls '
        'without human intervention. Navigation speed is regulated automatically by proximity to '
        'obstacles and path curvature.'))

    s.append(sub('2.5&nbsp;&nbsp;State estimation'))
    s.append(p(
        'An onboard estimator fuses leg kinematics with inertial measurement to estimate the '
        'robot&rsquo;s motion using no external reference &mdash; a prerequisite for real-world '
        'autonomy. Measured drift is 4&ndash;5% of distance travelled, within the normal band for '
        'kinematic legged odometry, and the mapping stack was shown to run successfully on this '
        'onboard estimate rather than on simulator ground truth.'))

    s.append(sub('2.6&nbsp;&nbsp;Embedded firmware and hardware interface'))
    s.append(p(
        'The communication protocol between the onboard computer and the real-time microcontroller '
        'was implemented and cross-verified in both languages against shared reference test '
        'vectors. The robot&rsquo;s motor-control interface was then validated against a software '
        'emulator that runs the exact microcontroller firmware logic: the entire stack &mdash; from '
        'gait through to serial telemetry &mdash; passes a nine-point integration test and walks '
        'with no hardware present. Commissioning the physical electronics is consequently reduced '
        'to changing a single device address; the firmware itself is written and compiles for the '
        'target microcontroller.'))

    s.append(sub('2.7&nbsp;&nbsp;Actuator load analysis'))
    s.append(p(
        'Using the calibrated digital twin, we measured the torque borne by each of the twelve '
        'servos throughout a normal walking cycle (Figure&nbsp;1). The continuous (RMS) load peaks '
        'at 1.31&nbsp;N&middot;m &mdash; 45% of the actuator&rsquo;s rated capacity, a continuous '
        'safety factor of 2.2&times;. Brief torque transients at foot-strike approach the rated '
        'limit, concentrated on the rear legs; these are partly an artefact of perfectly-rigid '
        'simulated contact and will be re-measured on the bench, but they identify where to watch '
        'actuator current first on hardware. The analysis confirms the selected servos are '
        'appropriately sized for the walking gait, with margin, and provides the baseline against '
        'which future control improvements will be judged.'))
    if os.path.exists(FIG):
        from reportlab.lib.utils import ImageReader
        iw, ih = ImageReader(FIG).getSize()
        w = 15.5 * cm
        img = Image(FIG, width=w, height=w * ih / iw)
        img.hAlign = 'CENTER'
        s.append(Spacer(1, 2 * mm))
        s.append(KeepTogether([img, Paragraph(
            'Figure 1. Per-servo torque during a normal trot (vx&nbsp;0.15&nbsp;m/s). Top: '
            'sustained torque vs. gait phase by joint class. Bottom: peak torque per servo '
            'against the 2.94&nbsp;N&middot;m actuator cap (dashed). Source: simulated joint '
            'transmitted-wrench, ground reaction included.', cap)]))

    # ── 3 quantitative results ──
    s.append(section('3', 'Quantitative results'))
    s.append(p('The table below collects the period&rsquo;s primary measured indicators. All '
               'figures are reproducible from the project&rsquo;s versioned tooling and data logs.'))
    rows = [
        ['Indicator', 'Result', 'Significance'],
        ['Model fidelity (settle-height error)', '0.2 mm', 'Simulation faithfully matches the modelled machine'],
        ['Joint tracking error (walking)', '17.8 mrad RMS', 'Actuator response calibrated to specification'],
        ['Autonomous mission', '16 m course completed', 'Final error 0.157 m; self-recovering'],
        ['Onboard odometry drift', '4&ndash;5% of distance', 'No external reference required'],
        ['Hardware-interface integration', '9 / 9 checks', 'Full stack runs without physical hardware'],
        ['Protocol codec tests', '6 / 6 + golden vectors', 'Computer&ndash;microcontroller link verified'],
        ['Control / kinematics test suite', '30 tests passing', 'Regression-protected'],
        ['Continuous actuator load', '45% of rated torque', '2.2&times; continuous safety margin'],
    ]
    s.append(Spacer(1, 2 * mm))
    s.append(metric_table(rows, [6.0 * cm, 3.4 * cm, 7.0 * cm]))

    # ── 4 engineering process ──
    s.append(section('4', 'Engineering process and risk management'))
    s.append(p(
        'Beyond technical results, the period produced durable process assets that reduce '
        'programme risk. Every engineering decision, change and experiment is recorded in a '
        'structured, version-controlled documentation system, with quantitative before/after '
        'metrics retained for eventual publication. The robot&rsquo;s power architecture has been '
        'specified and recorded (a single battery feeding a regulated motor supply, with a defined '
        'low-voltage protection ladder).'))
    s.append(p(
        'Most significantly, a complete execution plan &mdash; spanning calibration, mechanical '
        'assembly, firmware commissioning, first walking, perception, autonomy and machine '
        'learning, with measurable acceptance criteria and explicit contingency procedures at '
        'every step &mdash; has been authored as a self-contained reference. This plan is designed '
        'to be executed by a competent engineer without reliance on any single team member or tool, '
        'directly addressing key-person and continuity risk, and includes a fully specified '
        'fallback path to a deployable robot should the most advanced (machine-learning) approach '
        'prove unnecessary or unavailable.'))

    # ── 5 outlook ──
    s.append(section('5', 'Current status and next steps'))
    rows2 = [
        ['Phase', 'Status'],
        ['Simulation, control and inverse kinematics', 'Complete'],
        ['Physics-fidelity calibration to measured hardware', 'Complete'],
        ['Perception, SLAM and autonomous navigation', 'Complete (simulation)'],
        ['Onboard state estimation', 'Complete (simulation)'],
        ['Embedded protocol and hardware interface', 'Complete; validated on emulator'],
        ['Physical assembly and commissioning', 'Ready; awaiting components'],
        ['On-robot walking and autonomy', 'Planned; fully specified'],
        ['Learned locomotion policy', 'Planned; three compute paths + fallback'],
    ]
    s.append(metric_table(rows2, [10.5 * cm, 5.9 * cm]))
    s.append(Spacer(1, 3 * mm))
    s.append(p(
        'With components now arriving, the immediate next steps are per-servo bench calibration, '
        'mechanical assembly, and firmware commissioning &mdash; each with pre-defined pass/fail '
        'gates and contingency procedures. Because the entire software stack is already proven '
        'against an emulated controller, the transition to physical hardware is expected to be '
        'incremental rather than exploratory. We will report measured on-robot performance against '
        'the simulation baselines established in this period in the next update.'))
    s.append(Spacer(1, 4 * mm))
    s.append(Paragraph('&mdash; Aryaman Gupta &amp; Krish Agarwal', ParagraphStyle(
        'sig', parent=body, fontName=SERIF_I, alignment=TA_CENTER, textColor=ACCENT)))

    doc.build(s)
    print('wrote', OUT)


if __name__ == '__main__':
    build()
