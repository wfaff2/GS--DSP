from pathlib import Path
from typing import Union

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "figures"

PANELS = [
    ("a", ROOT / "figures_src" / "scenario_circle.png"),
    ("b", ROOT / "figures_src" / "scenario_clean_figure8.png"),
    ("c", ROOT / "figures_src" / "scenario_fixedobs_figure8.png"),
]

PANEL_W = 430
PANEL_H = 430
PANEL_PAD = 18
OUTER_PAD_X = 28
OUTER_PAD_Y = 28
GAP = 22
ROW_GAP = 24
LABEL_MARGIN = 14
LABEL_FONT_SIZE = 28
BORDER_COLOR = (220, 220, 220)
BACKGROUND = (255, 255, 255)
TEXT = (0, 0, 0)
RESAMPLE = getattr(Image, "LANCZOS", Image.ANTIALIAS)


def load_font(size: int) -> Union[ImageFont.FreeTypeFont, ImageFont.ImageFont]:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    ]
    for path in candidates:
        font_path = Path(path)
        if font_path.exists():
            return ImageFont.truetype(str(font_path), size=size)
    return ImageFont.load_default()


def fit_image(path: Path) -> Image.Image:
    image = Image.open(path).convert("RGB")
    image.thumbnail((PANEL_W - 2 * PANEL_PAD, PANEL_H - 2 * PANEL_PAD), RESAMPLE)
    panel = Image.new("RGB", (PANEL_W, PANEL_H), BACKGROUND)
    x = (PANEL_W - image.width) // 2
    y = (PANEL_H - image.height) // 2
    panel.paste(image, (x, y))
    draw = ImageDraw.Draw(panel)
    draw.rectangle([(0, 0), (PANEL_W - 1, PANEL_H - 1)], outline=BORDER_COLOR, width=1)
    return panel


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    font = load_font(LABEL_FONT_SIZE)
    canvas_w = OUTER_PAD_X * 2 + 2 * PANEL_W + GAP
    canvas_h = OUTER_PAD_Y * 2 + 2 * PANEL_H + ROW_GAP
    canvas = Image.new("RGB", (canvas_w, canvas_h), BACKGROUND)
    draw = ImageDraw.Draw(canvas)

    positions = [
        ((canvas_w - PANEL_W) // 2, OUTER_PAD_Y),
        (OUTER_PAD_X, OUTER_PAD_Y + PANEL_H + ROW_GAP),
        (OUTER_PAD_X + PANEL_W + GAP, OUTER_PAD_Y + PANEL_H + ROW_GAP),
    ]

    for (label, path), (x, y) in zip(PANELS, positions):
        panel = fit_image(path)
        canvas.paste(panel, (x, y))
        draw.text((x + LABEL_MARGIN, y + LABEL_MARGIN), f"({label})", fill=TEXT, font=font)

    png_path = OUT_DIR / "paper_fig1_scenarios.png"
    pdf_path = OUT_DIR / "paper_fig1_scenarios.pdf"
    canvas.save(png_path)
    canvas.save(pdf_path, "PDF", resolution=300.0)
    print(png_path)
    print(pdf_path)


if __name__ == "__main__":
    main()
