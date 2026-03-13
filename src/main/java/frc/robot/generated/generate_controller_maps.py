from PIL import Image, ImageDraw, ImageFont
import os

GENERATED = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(GENERATED, "xbox controller.jpg")

# Canvas: pad the image with white border so labels have room
PAD_LEFT   = 320
PAD_RIGHT  = 320
PAD_TOP    = 160
PAD_BOTTOM = 200

img_orig = Image.open(SRC).convert("RGB")
IW, IH = img_orig.size  # 1498 x 1043

CW = IW + PAD_LEFT + PAD_RIGHT
CH = IH + PAD_TOP  + PAD_BOTTOM

# Button positions relative to original image origin (0,0 = top-left of controller image)
# Measured visually on the 1498x1043 image.
# Shifted up by 8% of IH (~83px) to correct for observed offset.
Y_OFFSET = -int(IH * 0.08)

def B(x, y):
    return (x, y + Y_OFFSET)

# Button positions relative to original image origin (0,0 = top-left of controller image)
# Measured visually on the 1498x1043 image.
# Format: (x, y) of button center in original image coords.
BUTTONS = {
    # Face buttons: Y/A right 1.5%, B right 3%; Y up 3% extra
    "A":      B(1065 + int(IW*0.04) + int(IW*0.015), 480 - int(IH*0.05) + int(IH*0.06)),
    "B":      B(1118 + int(IW*0.04) + int(IW*0.030), 428 - int(IH*0.05) + int(IH*0.03)),
    "X":      B(1012 + int(IW*0.04),                 428 - int(IH*0.05) + int(IH*0.03)),
    "Y":      B(1065 + int(IW*0.04) + int(IW*0.015), 374 - int(IH*0.05) - int(IH*0.01) - int(IH*0.03)),
    # Bumpers: same Y
    "LB":     B( 368, 222 - int(IH*0.10)),
    "RB":     B(1128, 222 - int(IH*0.10)),
    # Triggers: up another 2% on top of previous -5%
    "LT":     B( 340, 138 - int(IH*0.05) - int(IH*0.02)),
    "RT":     B(1155, 138 - int(IH*0.05) - int(IH*0.02)),
    # Sticks: L left 10%, up 7%+8%=15% total
    "L_STICK": B(474 - int(IW*0.10), 468 - int(IH*0.15)),
    "R_STICK": B(878 + int(IW*0.05), 588),
    # D-pad: left 6%+2%=8% total; up/down/left/right all down 15% extra except DOWN which is down 3% more
    "DPAD_UP":    B(660 - int(IW*0.08), 375 + int(IH*0.02) + int(IH*0.15)),
    "DPAD_DOWN":  B(660 - int(IW*0.08), 460 + int(IH*0.02) + int(IH*0.12) + int(IH*0.15) + int(IH*0.03) - int(IH*0.05) - int(IH*0.04) - int(IH*0.015)),
    "DPAD_LEFT":  B(617 - int(IW*0.08), 418 + int(IH*0.02) + int(IH*0.15) + int(IH*0.02)),
    "DPAD_RIGHT": B(703 - int(IW*0.08), 418 + int(IH*0.02) + int(IH*0.15) + int(IH*0.02)),
    # Menu buttons — down 5%
    "BACK":  B(618, 310 + int(IH*0.05)),
    "START": B(878, 310 + int(IH*0.05)),
}

# Label definitions per controller.
# Each entry: button_key, label text, anchor side ("left" | "right" = which side the label sits on)
DRIVER_LABELS = [
    ("LT",        "LT: Smart zone sweep\n(alone)",                         "left"),
    ("RT",        "LT+RT: Heading snap\n(both triggers)",                   "right"),
    ("LB",        "LB: Bump traversal LEFT",                               "left"),
    ("RB",        "RB: Bump traversal RIGHT",                              "right"),
    ("Y",         "Y: Wall traversal FAR",                                 "right"),
    ("B",         "B: Wall traversal RIGHT",                               "right"),
    ("A",         "A: Wall traversal NEAR",                                "right"),
    ("X",         "X: Wall traversal LEFT",                                "left"),
    ("BACK",      "BACK: Reset field orientation",                         "left"),
    ("START",     "START: Seed pose (right corner)",                       "right"),
    ("DPAD_DOWN", "D-pad DOWN: Toggle QuestNav\nemergency mode",           "left"),
    ("L_STICK",   "Left stick: Drive\n(field-relative)",                   "left"),
    ("R_STICK",   "Right stick: Rotate",                                   "right"),
]

OPERATOR_LABELS = [
    ("LT",        "LT: Toggle flywheels on/off",                          "left"),
    ("RT",        "RT (hold): SHOOT\n(spindexer + singulator)",            "right"),
    ("Y",         "Y: Toggle intake extend/retract",                       "right"),
    ("B",         "B: Toggle intake rollers on/off",                       "right"),
    ("A",         "A: Agitate intake",                                     "right"),
    ("X",         "X (hold): Reverse rollers (spit)",                      "left"),
    ("DPAD_LEFT",  "D-pad LEFT (hold): Rotate turret CCW",                 "left"),
    ("DPAD_RIGHT", "D-pad RIGHT (hold): Rotate turret CW",                 "right"),
    ("DPAD_UP",    "D-pad UP: Hood step up",                               "left"),
    ("DPAD_DOWN",  "D-pad DOWN: Hood step down",                           "left"),
    ("BACK",      "BACK+START: Re-home turret\n(hall sweep only)",              "left"),
    ("START",     "(see BACK+START)",                                      "right"),
    ("LB",        "LB+RB: Confirm turret forward\n(enables all controls)", "left"),
    ("RB",        "(see LB+RB)",                                           "right"),
]

TITLE_DRIVER   = "RoboDominators 5142 - DRIVER Controller"
TITLE_OPERATOR = "RoboDominators 5142 - OPERATOR Controller"

# Try to load a decent font, fall back to default
def get_font(size):
    for name in ["arial.ttf", "Arial.ttf", "DejaVuSans.ttf"]:
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            pass
    return ImageFont.load_default()

FONT_LABEL = get_font(22)
FONT_TITLE = get_font(36)
FONT_SMALL = get_font(18)

DOT_R      = 8
LINE_COLOR = (255, 30, 30)      # bright red
TEXT_COLOR = (20, 20, 20)
DOT_COLOR  = (255, 220, 0)      # yellow
DOT_OUTLINE = (200, 160, 0)     # darker yellow outline
BG_COLOR   = (255, 255, 255)


def draw_label(draw, bx, by, text, side, pad_left, pad_top):
    """Draw leader line first, then dot on top so yellow is always above the red line."""
    cx = bx + pad_left
    cy = by + pad_top

    lines = text.split("\n")
    line_heights = []
    line_widths  = []
    for ln in lines:
        bb = draw.textbbox((0, 0), ln, font=FONT_LABEL)
        line_widths.append(bb[2] - bb[0])
        line_heights.append(bb[3] - bb[1])
    line_gap  = 4
    block_h   = sum(line_heights) + line_gap * (len(lines) - 1)
    block_w   = max(line_widths)

    TEXT_GAP = 3   # px gap between line endpoint and nearest text edge

    LEFT_TEXT_LEFT_EDGE  = int(CW * 0.02)
    RIGHT_TEXT_LEFT_EDGE = int(CW * (0.885 - 0.07 + 0.04))

    if side == "left":
        lx_start = LEFT_TEXT_LEFT_EDGE
        ty = cy - block_h // 2
        # Line stops 3px short of the text block's right edge
        draw.line([(cx, cy), (cx - 40, cy), (lx_start + block_w + TEXT_GAP, cy)], fill=LINE_COLOR, width=2)
        # Dot after line so it renders on top
        draw.ellipse([cx - DOT_R, cy - DOT_R, cx + DOT_R, cy + DOT_R],
                     fill=DOT_COLOR, outline=DOT_OUTLINE, width=2)
        for ln, lh in zip(lines, line_heights):
            draw.text((lx_start, ty), ln, fill=TEXT_COLOR, font=FONT_LABEL)
            ty += lh + line_gap
    else:
        lx_start = RIGHT_TEXT_LEFT_EDGE
        ty = cy - block_h // 2
        # Line stops 3px before the text block's left edge
        draw.line([(cx, cy), (cx + 40, cy), (lx_start - TEXT_GAP, cy)], fill=LINE_COLOR, width=2)
        # Dot after line so it renders on top
        draw.ellipse([cx - DOT_R, cy - DOT_R, cx + DOT_R, cy + DOT_R],
                     fill=DOT_COLOR, outline=DOT_OUTLINE, width=2)
        for ln, lh in zip(lines, line_heights):
            draw.text((lx_start, ty), ln, fill=TEXT_COLOR, font=FONT_LABEL)
            ty += lh + line_gap


def generate(labels, title, out_name):
    canvas = Image.new("RGB", (CW, CH), BG_COLOR)
    canvas.paste(img_orig, (PAD_LEFT, PAD_TOP))
    draw = ImageDraw.Draw(canvas)

    # Title
    tb = draw.textbbox((0, 0), title, font=FONT_TITLE)
    tw = tb[2] - tb[0]
    draw.text(((CW - tw) // 2, 30), title, fill=(10, 10, 10), font=FONT_TITLE)

    # Footer
    footer = "Team 5142 RoboDominators  |  WNE 2026"
    fb = draw.textbbox((0, 0), footer, font=FONT_SMALL)
    fw = fb[2] - fb[0]
    draw.text(((CW - fw) // 2, CH - 40), footer, fill=(120, 120, 120), font=FONT_SMALL)

    for (btn_key, text, side) in labels:
        if btn_key not in BUTTONS:
            continue
        bx, by = BUTTONS[btn_key]
        draw_label(draw, bx, by, text, side, PAD_LEFT, PAD_TOP)

    out_path = os.path.join(GENERATED, out_name)
    canvas.save(out_path, quality=95)
    print(f"Saved: {out_path}  ({CW}x{CH})")


generate(DRIVER_LABELS,   TITLE_DRIVER,   "controller_driver.jpg")
generate(OPERATOR_LABELS, TITLE_OPERATOR, "controller_operator.jpg")
