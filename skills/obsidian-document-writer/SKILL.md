---
name: obsidian-document-writer
description: Create or revise Markdown documents for this Obsidian knowledge base, preserving local properties, links, attachments, formulas, and repository directory conventions. Use when the user asks to write, restructure, or polish a knowledge note in this vault.
metadata:
  short-description: Write Obsidian-friendly Markdown notes for this vault
---
# Obsidian 文档生成与整理

为当前 `per-knowledge` 知识库创建或修改 Markdown 文档时使用。目标是让笔记同时适合 Obsidian 手机端阅读、VS Code 编辑、Codex 辅助整理和 GitHub 查看。

## 工作流程

1. 先读取目标文件、同目录相关笔记和项目根目录规则；确认是否有未提交修改。
2. 判断文档属于现有目录中的哪一类：`0-临时记录` 临时记录、`01-Papers` 论文、`03-Knowledges` 通用知识、`04-JobLogs` 项目日志、`05-life` 生活与个人学习。不要为了套用模板擅自新增或重命名目录。
3. 参考同主题笔记的写法；若目标文档已有 YAML properties，保留字段和现有值，只补缺失字段。新文档默认使用：

   ```yaml
   ---
   title: 文档标题
   date: YYYY-MM-DD
   tags: [主题标签]
   ---
   ```
4. 标题、properties 与正文保持顺序：YAML frontmatter → 一个一级标题 → 正文。若原文已有标题或 properties，不重复添加。
5. 先给结论和可执行步骤，再解释原理、条件、边界和待验证事项。长文使用清晰的二级、三级标题；每节围绕一个问题展开。
6. 使用通用 Markdown：代码块标注语言；行内公式用 `$...$`，行间公式用 `$$...$$`；表格不要过宽；图片和附件使用相对路径。
7. 跨笔记优先保留可被 VS Code、GitHub 和 Obsidian 共同读取的相对 Markdown 链接；可以在需要 Obsidian 导航时使用 `[[笔记名]]`，但不能把关键上下文只放在 Canvas、图形视图或插件动态结果中。
8. 将事实、工程推断、个人建议和待验证内容分开标识。技术内容保留适用条件、失败路径和来源；不要用默认值掩盖未知信息。
9. 修改完成后检查 `git diff`、`git status`、Markdown 空白和相对链接；不要编译、构建、安装插件或提交/推送，除非用户另行明确要求并授权。

## Properties 约定

- `title` 应是人类可读的完整标题，与一级标题一致或语义等价。
- `date` 记录首次创建或用户要求的文档日期；编辑旧文档时不要无故刷新日期。
- `tags` 使用短、稳定、可检索的主题词；已有标签优先保留，除非用户要求整理。
- 不自动添加无法确认的作者、来源、状态、别名或项目字段。
- Markdown 文档没有 frontmatter 时，只在用户明确要求“加 properties”、创建新文档，或任务明确要求统一元数据时补充；一般润色不借机批量改全库。

## Obsidian 适配原则

- 手机端优先：开头给出结论或用途，段落短，表格控制在少数列，避免必须横向滚动才能理解。
- 电脑端优先：保留稳定的标题层级、相对链接、可搜索的术语和语言标注代码块，方便 VS Code 全文搜索和 Codex 处理。
- 用核心功能优先解决导航：搜索、书签、大纲、反向链接、属性、Templates、Bases、Canvas。社区插件只在用户确实需要且已说明插件可用时使用。
- 新增链接或附件前，确认目标存在；移动或重命名文件后全库搜索旧路径。
- YAML 中避免未加引号的 `:`、`#`、`[`、`]` 等可能破坏解析的值；复杂文本用引号或拆成数组。

## 同步与 Git 边界

- 这套知识库由手机 Obsidian＋GitSync、电脑 VS Code＋Codex＋Git 共同编辑；避免建议两端同时修改同一篇长文。
- 写作任务默认只修改用户指定文件和为完成链接所必需的文件，不替换用户已有修改。
- 生成文档后报告实际修改文件、静态检查结果，以及尚未执行的编译或测试；文档任务通常无需编译。

详见 [Obsidian properties 与 Markdown 参考](references/obsidian-markdown.md)。
