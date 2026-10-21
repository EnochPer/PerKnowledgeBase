#!/usr/bin/env python3
"""切分、定位和记录《Multiple View Geometry》学习材料。

书中印刷页码与 PDF 物理页码分开处理。默认本书正文第 1 页对应 PDF 第 19 页，
但命令行参数允许覆盖这个偏移量。
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import tempfile
import sys
from functools import lru_cache
from datetime import date
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PDF = ROOT / "Multiple View Geometry in Computer Vision (Second Edition).pdf"
STATE = ROOT / "学习状态.json"
INDEX = ROOT / "页码索引.md"
TEXT_DIR = ROOT / "切分文本"
FIGURE_DIR = ROOT / "figures"

# 依据 PDF Contents 整理的正文起始页。页码是书中的印刷页码。
CHAPTERS = [
    ("第 1 章 导论", 1),
    ("第 2 章 二维射影几何与变换", 25),
    ("第 3 章 三维射影几何与变换", 65),
    ("第 4 章 二维射影变换的估计", 87),
    ("第 5 章 算法评价与误差分析", 132),
    ("第 6 章 相机模型", 153),
    ("第 7 章 相机矩阵 P 的计算", 178),
    ("第 8 章 更多单视图几何", 195),
    ("第 9 章 极线几何与基础矩阵", 239),
    ("第 10 章 相机与结构的三维重建", 262),
    ("第 11 章 基础矩阵 F 的计算", 279),
    ("第 12 章 结构计算", 310),
    ("第 13 章 场景平面与单应性", 325),
    ("第 14 章 仿射极线几何", 344),
    ("第 15 章 三焦张量", 365),
    ("第 16 章 三焦张量 T 的计算", 391),
    ("第 17 章 N 线性关系与多视图张量", 411),
    ("第 18 章 N 视图计算方法", 434),
    ("第 19 章 自动标定", 458),
    ("第 20 章 对偶性", 502),
    ("第 21 章 可见性与符号约束（Cheirality）", 515),
    ("第 22 章 退化配置", 533),
    ("附录 1 张量记号", 562),
    ("附录 2 高斯分布与 χ² 分布", 565),
    ("附录 3 参数估计", 568),
    ("附录 4 矩阵性质与分解", 578),
    ("附录 5 最小二乘最小化", 588),
    ("附录 6 迭代估计方法", 597),
    ("附录 7 特殊平面射影变换", 628),
]


def run(command: list[str]) -> str:
    completed = subprocess.run(command, check=True, text=True, capture_output=True)
    return completed.stdout


@lru_cache(maxsize=1)
def pdf_pages() -> int:
    output = run(["pdfinfo", str(PDF)])
    match = re.search(r"^Pages:\s+(\d+)", output, re.MULTILINE)
    if not match:
        raise RuntimeError("无法从 pdfinfo 读取页数")
    return int(match.group(1))


def physical_page(book_page: int, offset: int) -> int:
    if book_page < 1:
        raise ValueError("正文页码必须大于零")
    page = book_page + offset
    if page < 1 or page > pdf_pages():
        raise ValueError(f"书中页码 {book_page} 映射到 PDF 页 {page}，超出 PDF 范围")
    return page


def chapter_for(book_page: int) -> str:
    if book_page >= 634:
        return "参考文献与索引"
    current = CHAPTERS[0][0]
    for name, start in CHAPTERS:
        if book_page >= start:
            current = name
        else:
            break
    return current


def page_link(book_page: int, offset: int) -> str:
    page = physical_page(book_page, offset)
    filename = PDF.name.replace(" ", "%20").replace("(", "%28").replace(")", "%29")
    return f"./{filename}#page={page}"


def ensure_dirs() -> None:
    TEXT_DIR.mkdir(exist_ok=True)
    FIGURE_DIR.mkdir(exist_ok=True)


def cmd_index(args: argparse.Namespace) -> None:
    lines = [
        "# 《Multiple View Geometry》页码索引",
        "",
        f"> 书中印刷页码 + {args.offset} = PDF 物理页码。当前 PDF 共 {pdf_pages()} 页。",
        "",
        "| 章节 | 原书起始页 | PDF 页 | 跳转 |",
        "|---|---:|---:|---|",
    ]
    for name, start in CHAPTERS:
        page = physical_page(start, args.offset)
        lines.append(f"| {name} | {start} | {page} | [打开 PDF]({page_link(start, args.offset)}) |")
    INDEX.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(INDEX)


def cmd_extract(args: argparse.Namespace) -> None:
    ensure_dirs()
    first = physical_page(args.start, args.offset)
    last = physical_page(args.end, args.offset)
    if first > last:
        raise ValueError("起始页不能大于结束页")
    output = TEXT_DIR / f"原书{args.start}-{args.end}页_物理{first}-{last}页.txt"
    text = run(["pdftotext", "-f", str(first), "-l", str(last), "-layout", str(PDF), "-"])
    output.write_text(text, encoding="utf-8")
    print(output)


def cmd_chapter(args: argparse.Namespace) -> None:
    if args.chapter < 1 or args.chapter > len(CHAPTERS):
        raise ValueError(f"章节编号必须在 1 到 {len(CHAPTERS)} 之间")
    _, start = CHAPTERS[args.chapter - 1]
    end = CHAPTERS[args.chapter][1] - 1 if args.chapter < len(CHAPTERS) else 633
    args.start, args.end = start, end
    cmd_split(args) if args.pdf else cmd_extract(args)


def cmd_split(args):
    first = physical_page(args.start, args.offset)
    last = physical_page(args.end, args.offset)
    if first > last:
        raise ValueError("起始页不能大于结束页")
    folder = ROOT / "切分PDF"
    folder.mkdir(exist_ok=True)
    output = folder / f"原书{args.start}-{args.end}页.pdf"
    if output.exists():
        raise FileExistsError(output)
    with tempfile.TemporaryDirectory() as tmp:
        run(["pdfseparate", "-f", str(first), "-l", str(last), str(PDF), tmp + "/page-%d.pdf"])
        run(["pdfunite", *[f"{tmp}/page-{n}.pdf" for n in range(first, last+1)], str(output)])
    print(output)


def cmd_locate(args: argparse.Namespace) -> None:
    print(json.dumps({
        "chapter": chapter_for(args.book_page),
        "book_page": args.book_page,
        "pdf_page": physical_page(args.book_page, args.offset),
        "link": page_link(args.book_page, args.offset),
    }, ensure_ascii=False, indent=2))


def cmd_figures(args: argparse.Namespace) -> None:
    """提取页面内嵌位图；矢量图页面则保留整页渲染图。"""
    FIGURE_DIR.mkdir(exist_ok=True)
    physical = physical_page(args.book_page, args.offset)
    prefix = FIGURE_DIR / f"原书{args.book_page}页_物理{physical}页_图"
    run(["pdfimages", "-f", str(physical), "-l", str(physical), "-png", str(PDF), str(prefix)])
    images = sorted(FIGURE_DIR.glob(prefix.name + "-*.png"))
    if not images:
        args.dpi = 180
        cmd_render(args)
        print("页面未包含可直接提取的嵌入位图，已生成整页渲染图作为保底。")
    else:
        for image in images:
            print(image)


def cmd_render(args: argparse.Namespace) -> None:
    FIGURE_DIR.mkdir(exist_ok=True)
    physical = physical_page(args.book_page, args.offset)
    prefix = FIGURE_DIR / f"原书{args.book_page}页_物理{physical}页"
    command = ["pdftoppm", "-f", str(physical), "-l", str(physical), "-png", "-singlefile"]
    if args.dpi <= 0:
        raise ValueError("DPI 必须大于零")
    if args.dpi:
        command += ["-r", str(args.dpi)]
    if getattr(args, "crop", None):
        x, y, w, h = args.crop
        if min(x, y) < 0 or min(w, h) <= 0:
            raise ValueError("裁剪坐标须非负，宽高须为正")
        command += ["-x", str(x), "-y", str(y), "-W", str(w), "-H", str(h)]
        prefix = prefix.with_name(prefix.name + f"_crop_{x}_{y}_{w}_{h}")
    command += [str(PDF), str(prefix)]
    run(command)
    print(f"{prefix}.png")


def load_state() -> dict:
    if STATE.exists():
        state = json.loads(STATE.read_text(encoding="utf-8"))
        for field in ("organized_topics", "mastered_topics", "pending_questions", "notes"):
            if not isinstance(state.get(field), list):
                raise ValueError(f"学习状态字段 {field} 缺失或不是列表")
        return state
    return {
        "updated": str(date.today()),
        "current": {"chapter": "第 1 章 导论", "book_page": 1, "pdf_page": 19},
        "organized_topics": [],
        "mastered_topics": [],
        "pending_questions": [],
        "notes": ["01_综述.md"],
    }


def cmd_status(args: argparse.Namespace) -> None:
    state = load_state()
    print(json.dumps(state, ensure_ascii=False, indent=2))


def cmd_set(args: argparse.Namespace) -> None:
    state = load_state()
    state["updated"] = str(date.today())
    state["current"] = {
        "chapter": chapter_for(args.book_page),
        "book_page": args.book_page,
        "pdf_page": physical_page(args.book_page, args.offset),
    }
    if args.topic and args.topic not in state["organized_topics"]:
        state["organized_topics"].append(args.topic)
    if args.mastered and args.mastered not in state["mastered_topics"]:
        state["mastered_topics"].append(args.mastered)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=STATE.parent, delete=False) as tmp:
        tmp.write(json.dumps(state, ensure_ascii=False, indent=2) + "\n")
        staged = Path(tmp.name)
    staged.replace(STATE)
    print(STATE)


def parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--offset", type=int, default=18, help="书中页码到 PDF 页码的偏移量")
    root = argparse.ArgumentParser(description=__doc__)
    sub = root.add_subparsers(dest="command", required=True)
    index = sub.add_parser("index", parents=[common], help="生成章节页码索引")
    index.set_defaults(func=cmd_index)
    extract = sub.add_parser("extract", parents=[common], help="提取一段书中页码的文本")
    extract.add_argument("start", type=int)
    extract.add_argument("end", type=int)
    extract.set_defaults(func=cmd_extract)
    chapter = sub.add_parser("chapter", parents=[common], help="按章节编号提取文本")
    chapter.add_argument("chapter", type=int)
    chapter.add_argument("--pdf", action="store_true", help="输出独立 PDF，默认输出文本")
    chapter.set_defaults(func=cmd_chapter)
    split = sub.add_parser("split", parents=[common], help="按原书页范围切分 PDF")
    split.add_argument("start", type=int)
    split.add_argument("end", type=int)
    split.set_defaults(func=cmd_split)
    locate = sub.add_parser("locate", parents=[common], help="输出原书页、PDF 页和跳转链接")
    locate.add_argument("book_page", type=int)
    locate.set_defaults(func=cmd_locate)
    figures = sub.add_parser("figures", parents=[common], help="提取一页中的嵌入插图")
    figures.add_argument("book_page", type=int)
    figures.set_defaults(func=cmd_figures)
    render = sub.add_parser("render", parents=[common], help="渲染一页书中页面为 PNG")
    render.add_argument("book_page", type=int)
    render.add_argument("--dpi", type=int, default=150)
    render.add_argument("--crop", type=int, nargs=4, metavar=("X", "Y", "W", "H"), help="所选 DPI 下从页面左上角计的像素裁剪框")
    render.set_defaults(func=cmd_render)
    status = sub.add_parser("status", help="显示学习状态")
    status.set_defaults(func=cmd_status)
    set_command = sub.add_parser("set", parents=[common], help="更新当前学习位置")
    set_command.add_argument("book_page", type=int)
    set_command.add_argument("--topic")
    set_command.add_argument("--mastered")
    set_command.set_defaults(func=cmd_set)
    return root


if __name__ == "__main__":
    arguments = parser().parse_args()
    try:
        arguments.func(arguments)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(getattr(error, "stderr", None) or str(error), file=sys.stderr)
        sys.exit(1)
