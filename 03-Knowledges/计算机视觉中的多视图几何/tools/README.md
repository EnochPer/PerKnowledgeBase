# 学习工具使用说明

在本 `tools` 目录执行以下命令。只依赖 Python 标准库和系统已有 Poppler 工具；不安装依赖、不构建。使用 `-B` 禁止生成 Python 字节码缓存。

```bash
python3 -B mvg_pdf.py locate 154
python3 -B mvg_pdf.py extract 153 154
python3 -B mvg_pdf.py chapter 6
python3 -B mvg_pdf.py chapter 6 --pdf
python3 -B mvg_pdf.py split 153 154
python3 -B mvg_pdf.py render 154 --dpi 150
python3 -B mvg_pdf.py render 154 --dpi 150 --crop 220 155 875 325
python3 -B mvg_pdf.py figures 154
python3 -B mvg_pdf.py index
python3 -B mvg_state.py status
python3 -B mvg_state.py set 154 --topic '针孔投影'
```

所有页码参数都是**原书印刷页码**；当前版本正文 PDF 物理页码 = 原书页码 + 18。分章编号 1–22 对应正文，23–29 对应附录 1–7。分章区间截至下一章前一页，可能含部标题页或空白页。切分 PDF 内部页码从 1 开始，笔记回链始终指向完整原书。

输出分别位于 `切分文本/`、`切分PDF/`、`figures/`。重复切分同一范围 PDF 会拒绝覆盖；同名文本、渲染图及页码索引是可重新生成的文件，请勿手工编辑它们。`figures` 只能提取内嵌位图，未找到时渲染整页；带标注的矢量几何图应优先使用 `render --crop`。裁剪框是选定 DPI 下从页面左上角计的 X、Y、宽、高，需先看整页确定，不能套用于其他页。

`set` 仅切换当前位置，`--topic` 仅表示笔记已整理。只有用户确认理解后才使用 `--mastered '主题'`；状态中保留待复习问题、笔记路径和恢复提示。状态不存在时从第 1 章初始化，格式损坏时明确报错。

每次恢复会话：

1. 读取 `学习状态.json` 与对应笔记，确认用户的问题。
2. 用 `locate` 找页码；用 `extract` 只读有关几页，不读取全书。
3. 查图与公式时渲染该页，确认图号、公式号和符号，再裁图引用。
4. 笔记注明章节/小节、印刷页、PDF 页及图号/公式号；自拟数值例子注明是讲解示例。
5. 更新笔记与学习位置，记录未解决问题；不要自动把“已整理”当成“已掌握”。

PDF 链接采用 `文件.pdf#page=N`。是否在点击后跳到指定页取决于 Markdown/PDF 查看器；若查看器忽略片段，请按笔记所列物理页跳转。链接和页面映射已检查，不代表每个 IDE 查看器都已验证。
