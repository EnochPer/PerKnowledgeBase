# T\-RO综述解读：Foundation Model时代机器人操作的知识版图

> 部分内容由豆包生成
> 
> 

本文基于机器之心公众号文章《T\-RO综述重构Foundation Model时代机器人操作知识版图》（2026\-09\-04），总结其核心内容，并检索原论文及相关工作进行深度分析与最新进展补充。原论文已被 IEEE Transactions on Robotics（T\-RO）接收。

# 一、文章与论文概览

## （一）公众号文章信息

- **来源**：机器之心 · AIxiv 专栏

- **发布时间**：2026 年 9 月 4 日

- **原文链接**：[微信公众号原文](https://mp.weixin.qq.com/s/OkR2YzgKarWogjFP6V43yA)

- **文章主旨**：解读一篇被 T\-RO 接收的综述，指出 LLM Planner、VLA、Diffusion Policy、Imitation Learning 等热门概念并不处在同一坐标轴上，需要用"系统层级"重新组织机器人操作研究。

## （二）原论文元信息

|项目|内容|
|---|---|
|论文标题|Embodied Robot Manipulation in the Era of Foundation Models: Planning and Learning Perspectives|
|发表状态|IEEE Transactions on Robotics（T\-RO）接收|
|arXiv|[arXiv:2512\.22983](https://arxiv.org/abs/2512.22983)（v1 提交于 2025\-12\-28，v2 修订于 2026\-08\-15）|
|第一作者|白双豪（西安交通大学）、宋文轩（香港科技大学广州）|
|通讯作者|李昊昂（香港科技大学广州）、仉尚航（北京大学）、陈霸东（西安交通大学）|
|合作机构|西安交通大学、香港科技大学（广州）、中国科学院、西湖大学、浙江大学、悉尼大学、北京智源人工智能研究院、北京大学等|
|作者总数|17 位|
|项目仓库|[github\.com/BaiShuanghao/Awesome\-Robotics\-Manipulation](https://github.com/BaiShuanghao/Awesome-Robotics-Manipulation)|

## （三）重要补充：核心版与完整版的关系

公众号未提及的一个关键信息是：这篇 T\-RO 论文并非该团队的唯一版本。arXiv 页面的 Comments 字段明确写道：

> "This work is a re\-architected core derived from the full survey \(arXiv:2510\.10903\), refined to highlight the most central themes and representative studies\."
> 
> 

也就是说，**arXiv:2512\.22983（T\-RO 版）是从更完整的综述 arXiv:2510\.10903《Towards a Unified Understanding of Robot Manipulation: A Comprehensive Survey》中重构出的核心版**。

[2510\.10903v2\.pdf](图片和附件/2510.10903v2.pdf)

完整版范围更广，额外包含了数据收集/利用/泛化瓶颈的专门分类、以及真实世界应用的系统综述；T\-RO 核心版则聚焦 planning 与 learning 两条主线，篇幅更精炼、论证更集中。阅读时可根据需求选择：想快速把握框架读 T\-RO 版，想全面了解数据与应用读完整版。

# 二、公众号内容要点总结

## （一）一张图重新理解机器人操作的系统边界

论文将机器人操作过程抽象为三个层级，分别回答三个不同的问题：

这三层提供了一套重新理解热门概念的坐标系：

|常见概念|它主要回答的问题|所属层级|
|---|---|---|
|LLM / MLLM Planner|如何形成高层任务计划和结构化指导？|High\-Level Planning|
|VLA|如何利用视觉、语言等多模态信息建模机器人动作？|Low\-Level（Input Modeling）|
|Latent Action|感知与动作之间能否学习更紧凑的表示？|Low\-Level（Latent Learning）|
|Diffusion / Flow Matching|动作或轨迹应该如何生成？|Low\-Level（Policy Learning）|
|Imitation / Reinforcement Learning|模型通过什么监督或反馈信号学习？|学习策略（跨层级）|

**核心观点**：VLA、Diffusion Policy、Imitation Learning 并不是三个平行的"机器人路线"，它们在回答不同维度的问题。

## （二）Foundation Model 真正改变了规划中的什么？

Foundation Model 对规划最直接的影响，不是让 LLM 直接输出关节控制量，而是让机器人拥有了更强的语义理解、开放世界知识和多模态推理能力。论文将这一过程理解为生成不同形式的 **planning artifacts**（规划产物）：

|Planning Artifact|Foundation Model 提供什么|下游如何使用|
|---|---|---|
|Task Plan|任务分解、技能排序、子目标|技能调用与进一步规划|
|Program|条件、循环和任务逻辑|调用感知与控制接口|
|Geometric Constraint|位姿、关键点和空间关系|运动规划与轨迹优化|
|Affordance Prior|可交互区域与功能关系|抓取、接触与规划|
|3D Representation|空间对应、抓取候选、优化目标|规划与动作生成|

一个典型矛盾是"语言上合理"不等于"物理上可行"。SayCan、Inner Monologue 展示了 LLM 的任务分解能力，但机器人生活在连续物理空间中。于是 VoxPoser 将语言理解转化为空间 value map，ReKep 将操作任务表示为关键点关系约束，GeoManip 把几何约束作为语义推理与运动生成之间的接口——Foundation Model 不负责完整轨迹，而是回答"什么样的空间关系应该被满足"，轨迹仍交给经典运动规划器。

## （三）VLA、Diffusion Policy 和 Imitation Learning，为什么不是一回事？

低层动作建模被拆成三个问题：**Input Modeling → Latent Learning → Policy Learning**。

1. **Input Modeling（基于什么信息产生动作）**：从 RGB/多视角视觉，到深度、点云、3D 表示，再到自然语言任务条件——VLA 的重要变化正发生在这里，它使视觉、语言、机器人状态进入同一个动作建模过程。输入边界还在扩展到触觉、力觉、声音。

2. **Latent Learning（感知与动作之间需要怎样的中间空间）**：一类用通用图像/人类视频/机器人交互数据预训练视觉或多模态表示；另一类进入动作本身，把连续运动压缩为离散 action token 或连续 latent vector，甚至与 world model 结合。

3. **Policy Learning（最终动作怎么生成）**：从 MLP 回归，到 Transformer 序列建模，到 Diffusion Policy 处理多峰动作分布，再到近年的 Flow Matching。

因此：VLA 描述"输入和动作如何组织"，Diffusion/Flow Matching 描述"动作如何生成"，IL/RL 描述"模型如何学习"——它们是正交维度，一个 VLA 完全可以采用 Transformer、Diffusion 或 Flow Matching 作为动作生成模块。

## （四）通用 Robot Brain 真正还缺什么？

1. **真正的外推能力**：当前很多泛化仍接近训练分布上的插值，通用机器人需要面对全新任务结构、场景、物体乃至本体，并持续学习。

2. **数据和评测**：真实机器人数据昂贵、采集慢，CALVIN、LIBERO、SimplerEnv 等平台各覆盖一部分能力，缺少统一可复现的评测体系。

3. **真正的物理感知**：对布料、线缆、流体、颗粒物，仅靠视觉不够，需要触觉、力觉、声音及对材料和动力学的理解。

4. **安全与协作**：进入家庭/工厂后，成功率不是唯一指标，需要持续监控、失败恢复、不确定时降风险，并满足运动学/动力学/接触力/能量等安全约束。

## （五）从"模型竞赛"重新回到机器人系统

论文没有创造新的模型家族，而是回到机器人系统本身，以 High\-Level Planning 和 Low\-Level Action Modeling 为主线，把快速出现的方法放回它们真正解决的问题中。未来真实机器人系统很可能不会简单地"端到端取代一切"——学习模型提供泛化和适应，规则、规划器和 MPC 等传统方法仍在可靠性和安全性上发挥作用。

# 三、原论文深度分析与补充

## （一）论文定位：与其他综述的差异

论文在引言中明确了自身的差异化定位：**近期综述往往按特定模型家族组织（VLA 综述、Diffusion 综述、生成式方法综述），而本综述采用 planning 与 learning 的统一抽象，将这些方法视为高层规划机制、低层学习策略或其组合的不同实例**。这不是简单的分类调整，而是一个方法论选择：当领域概念膨胀时，按"解决什么问题"而非"叫什么名字"组织文献，才能避免把系统层级、模型结构、动作生成方式和学习策略混为一谈。

这一定位也解释了为什么论文的核心贡献不是"覆盖了多少篇论文"，而是提供了一套可复用的分析坐标系——读者可以用它来判断任何一篇新论文究竟在解决哪个维度的问题。

## （二）官方摘要与方法论框架（精确表述）

以下为论文官方摘要的关键表述（译自 arXiv 摘要），比公众号转述更精确：

> "本综述从算法视角审视机器人操作，通过高层规划与低层动作建模的统一抽象组织近期的学习型方法。在高层，我们将经典任务规划概念扩展到包括对语言、代码、affordance、几何约束和 3D 表示的推理；在低层，我们提出面向学习范式的动作模型分类法，涵盖输入建模、潜在表示学习和策略学习。在这一抽象下，基础模型要么通过生成结构化规划产物（被实例化为下游动作生成的约束或潜在输入）做出贡献，要么直接建模可执行的动作和轨迹。最后，我们总结了与可扩展性、泛化、数据效率、多模态物理交互和安全性相关的开放挑战与未来方向。"
> 
> 

注意两个精确表述：①低层分类法是 **"learning\-paradigm\-oriented"**（面向学习范式），不是按模型名称；②Foundation Model 的贡献被明确分为**两种路径**——生成规划产物（间接）或直接建模动作（直接），这比公众号的"生成 planning artifacts"表述更完整。

## （三）High\-Level Planning 技术脉络深化

公众号提到了 SayCan、VoxPoser、ReKep、GeoManip，以下补充各工作的核心机制与差异：

|工作|Planning Artifact 类型|核心机制|特点|
|---|---|---|---|
|SayCan / Inner Monologue|Task Plan|LLM 给出技能序列，用价值函数/可完成性评分接地|长时序任务分解强，但依赖预定义技能原语|
|Code as Policies \(Liang et al\., 2022\)|Program|LLM 直接写 Python 策略代码，调用感知与控制 API|表达条件/循环/反馈，空间几何推理可复用第三方库|
|VoxPoser \(Huang et al\., 2023\)|3D Representation / Value Map|LLM 写代码与 VLM 交互，组合 3D value map，模型规划器零样本合成轨迹|开放指令\+开放物体，闭环抗扰动|
|ReKep \(Huang et al\., 2024\)|Geometric Constraint（关键点）|任务表示为 3D 关键点关系约束（Python 函数→数值代价），分层优化求解 SE\(3\) 动作|免手动标注、实时感知\-动作循环，大模型自动从语言生成 ReKep|
|GeoManip \(Tang et al\., 2025\)|Geometric Constraint（通用接口）|约束生成器\+几何解析器，求解器优化轨迹满足约束；training\-free|in\-context 学习，支持策略自适应、从演示/失败学习、长时序规划|

这条脉络的演进逻辑清晰：从"语言→技能序列"（SayCan），到"语言→可执行代码"（Code as Policies），到"语言→3D 空间价值图"（VoxPoser），到"语言→关键点几何约束"（ReKep），再到"几何约束作为通用接口"（GeoManip）。**趋势是 Foundation Model 的输出越来越物理接地、越来越可被经典规划器直接消费**，而不是让 LLM 直接输出动作。

## （四）Low\-Level Action Modeling 技术脉络深化

### 1\. Input Modeling：VLA 的演进

VLA（Vision\-Language\-Action）是低层输入建模的核心范式。代表性工作：

- **RT\-2 / RT\-2\-X**（Google DeepMind）：将视觉\-语言模型直接微调为输出动作 token，证明互联网规模知识可迁移到机器人。

- **OpenVLA**（Kim et al\., 2024）：7B 参数开源 VLA，基于 Llama 2 \+ DINOv2/SigLIP 视觉编码器，在 97 万条真实机器人演示上训练；在 29 个任务上比 55B 的 RT\-2\-X 绝对成功率高 16\.5%，参数少 7 倍；支持消费级 GPU 微调。

- **GR00T N1**（NVIDIA, 2025）：面向人形机器人的开源 VLA，双系统架构——System 2（视觉\-语言模块）理解环境，System 1（Diffusion Transformer）实时生成流畅动作；在 Fourier GR\-1 人形上验证双臂操作。

### 2\. Latent Learning：从感知预训练到动作潜在空间

- **LAPA**（Ye et al\., CoRL 2024）：首个无需机器人动作标签的 VLA 预训练方法，用 VQ\-VAE 从视频帧间学习离散潜在动作，再用互联网规模视频预训练，最后小样本机器人数据微调；在语言条件、未见物体泛化上超过有标签 VLA。

- **LatentVLA**（Wang et al\., AAAI）：学习连续且时空接地的潜在动作表示，用 Temporal\-Attentive Latent Action Model 解决"视觉相似状态需不同动作"的歧义，再在连续潜在空间中做 Diffusion 规划，保留运动保真度而无需 token 化。

### 3\. Policy Learning：生成范式的迭代

- **Diffusion Policy**：通过去噪扩散处理复杂多峰动作分布，已成为操作策略的主流生成范式。

- **Flow Matching**：近年兴起的动作生成范式，训练更稳定、采样更快，正在被新一代 VLA 采用。

- **LLaDA\-VLA**（Wen et al\., 2025）：首个基于掩码扩散 VLM（d\-VLM）的 VLA，用局部特殊 token 分类\+分层动作结构解码，在仿真和真实机器人上超过现有 VLA。

- **Dream\-VLA**（Ye et al\., 2025）：基于扩散语言模型（dLLM）的 VLA，双向生成天然适合 action chunking 和并行生成，在 LIBERO 上 97\.2%、SimplerEnv 上超过 π0 和 GR00T\-N1。

### 4\. 多模态物理感知的补充

公众号提到触觉、力觉、声音开始进入动作模型，最新工作 **FuSe**（Jones et al\., 2025）给出了具体方案：用自然语言作为跨模态公共接地，通过多模态对比损失\+感官接地语言生成损失，让通用策略在缺少大规模触觉/声音数据集时也能微调；在需要视觉\+触觉\+声音联合推理的任务上，零样本成功率比基线高 20% 以上。这直接回应了论文"真正的物理感知"缺口。

## （五）论文接收后的最新进展（2025 下半年—2026）

论文 v2 修订于 2026 年 8 月，但以下工作代表了论文框架提出后的最新趋势，可作为延伸阅读：

|工作|时间|核心贡献|对应论文维度|
|---|---|---|---|
|GR\-RL|2025\.12|将通用 VLA 策略通过 RL 专精为长时序灵巧操作专家，首次实现自主系鞋带（83\.3% 成功率），需毫米级精度和软体交互|Policy Learning \+ RL|
|VideoVLA|NeurIPS 2025|将视频生成模型直接转为 VLA，同时预测动作序列和未来视觉结果，高质量"想象"与可靠动作正相关|Latent Learning \+ World Model|
|DreamVLA|2025\.07|引入动态区域引导的世界知识预测（动态\+空间\+语义），块结构化注意力防信息泄漏，真实任务 76\.7% 成功率|Latent Learning|
|LLaDA\-VLA|2025\.09|首个掩码扩散 VLM 架构的 VLA，证明双向扩散骨干优于自回归|Policy Learning|
|FuSe|2025\.01|语言接地实现异构传感器（触觉/声音/视觉）微调通用策略|Input Modeling（多模态）|

这些最新工作验证了论文框架的预测性：**动作生成范式正在从 Diffusion 向 Flow Matching/掩码扩散演进；潜在表示学习与世界模型结合更紧密；多模态物理感知从"讨论缺口"走向"有方案"；VLA\+RL 的专精化成为从通用到可靠的路径**。

# 四、核心概念辨析：一张表看懂"谁在回答什么问题"

|概念|回答的问题|所属维度|与其他概念的关系|
|---|---|---|---|
|LLM Planner|做什么（任务分解/子目标）|High\-Level Planning|输出可被 VLA 或规划器消费|
|VLA|视觉\+语言如何组织成动作输入|Low\-Level: Input Modeling|可搭配任意 Policy Learning 方法|
|Latent Action|感知\-动作间的紧凑中间表示|Low\-Level: Latent Learning|可嵌入 VLA，也可独立预训练|
|Diffusion Policy|动作如何生成（去噪过程）|Low\-Level: Policy Learning|可作为 VLA 的动作头|
|Flow Matching|动作如何生成（向量场）|Low\-Level: Policy Learning|Diffusion 的替代/演进|
|World Model|预测未来状态/动力学|跨 Latent/Policy|可为规划提供想象空间|
|Imitation Learning|从演示中学习（监督信号）|学习策略|可训练上述任意模型|
|Reinforcement Learning|从奖励/交互中学习（反馈信号）|学习策略|可在 IL 预训练后专精化|

**一句话记忆**：VLA 管"看什么\+说什么怎么进模型"，Latent 管"中间怎么压缩"，Diffusion/Flow 管"动作怎么吐出来"，IL/RL 管"用什么信号学"——四者正交，可自由组合。

# 五、关键文献索引

## （一）核心综述

1. Bai S\., Song W\., et al\. Embodied Robot Manipulation in the Era of Foundation Models: Planning and Learning Perspectives\. IEEE T\-RO, 2026\. [arXiv:2512\.22983](https://arxiv.org/abs/2512.22983)

2. Bai S\., Song W\., et al\. Towards a Unified Understanding of Robot Manipulation: A Comprehensive Survey \(完整版\)\. 2025\. [arXiv:2510\.10903](https://arxiv.org/abs/2510.10903)

## （二）High\-Level Planning 代表工作

1. Liang J\., Huang W\., et al\. Code as Policies: Language Model Programs for Embodied Control\. 2022\. [arXiv:2209\.07753](https://arxiv.org/abs/2209.07753)

2. Huang W\., Wang C\., et al\. VoxPoser: Composable 3D Value Maps for Robotic Manipulation with Language Models\. CoRL 2023\. [arXiv:2307\.05973](https://arxiv.org/abs/2307.05973)

3. Huang W\., Wang C\., et al\. ReKep: Spatio\-Temporal Reasoning of Relational Keypoint Constraints\. 2024\. [arXiv:2409\.01652](https://arxiv.org/abs/2409.01652)

4. Tang W\., Pan J\., et al\. GeoManip: Geometric Constraints as General Interfaces for Robot Manipulation\. 2025\. [arXiv:2501\.09783](https://arxiv.org/abs/2501.09783)

## （三）Low\-Level Action Modeling 代表工作

1. Kim M\.J\., Pertsch K\., et al\. OpenVLA: An Open\-Source Vision\-Language\-Action Model\. 2024\. [arXiv:2406\.09246](https://arxiv.org/abs/2406.09246)

2. Ye S\., Jang J\., et al\. Latent Action Pretraining From Videos \(LAPA\)\. CoRL 2024\. [arXiv:2410\.11758](https://arxiv.org/abs/2410.11758)

3. NVIDIA\. GR00T N1: An Open Foundation Model for Generalist Humanoid Robots\. 2025\. [arXiv:2503\.14734](https://arxiv.org/abs/2503.14734)

4. Jones J\., Mees O\., et al\. Beyond Sight: Finetuning Generalist Robot Policies with Heterogeneous Sensors \(FuSe\)\. 2025\. [arXiv:2501\.04693](https://arxiv.org/abs/2501.04693)

5. Wen Y\., Li H\., et al\. LLaDA\-VLA: Vision Language Diffusion Action Models\. 2025\. [arXiv:2509\.06932](https://arxiv.org/abs/2509.06932)

6. Ye J\., Gong S\., et al\. Dream\-VL \& Dream\-VLA: Open VLM/VLA with Diffusion Language Model Backbone\. 2025\. [arXiv:2512\.22615](https://arxiv.org/abs/2512.22615)

## （四）最新进展（论文接收后）

1. GR\-RL: Going Dexterous and Precise for Long\-Horizon Robotic Manipulation\. 2025\. [arXiv:2512\.01801](https://arxiv.org/abs/2512.01801)

2. Shen Y\., Wei F\., et al\. VideoVLA: Video Generators Can Be Generalizable Robot Manipulators\. NeurIPS 2025\.

3. Zhang W\., et al\. DreamVLA: A VLA Model Dreamed with Comprehensive World Knowledge\. 2025\. [arXiv:2507\.04447](https://arxiv.org/abs/2507.04447)

4. Wang J\. LatentVLA: Taming Latent Space for Generalizable Long\-Horizon Bimanual Manipulation\. AAAI 2025\.

# 六、阅读建议与延伸

1. **入门路径**：先读公众号文章建立三层框架直觉 → 读 T\-RO 核心版（2512\.22983）把握方法论 → 按需读完整版（2510\.10903）的数据与应用章节。

2. **动手实践**：OpenVLA 是目前最易复现的开源 VLA（7B、消费级 GPU 可微调），适合作为 Low\-Level Action Modeling 的实验起点；高层规划可从 Code as Policies 或 VoxPoser 的开源代码入手。

3. **跟踪前沿**：关注论文的 GitHub Awesome 列表（持续更新），以及 Diffusion→Flow Matching/掩码扩散的动作生成范式迁移、VLA\+RL 专精化、多模态物理感知三个方向。

4. **批判性视角**：论文框架的价值在于"澄清概念"，但它本身不解决任何一个具体缺口；阅读时应追问"这篇新论文在框架的哪个格子里？比格子里已有工作好在哪里？"，避免被模型名称带偏。

---

*本文档由公众号内容总结 \+ 原论文检索分析 \+ 相关工作补充整合而成，所有论文链接均来自 arXiv 或官方出版页面，可直接访问。*

> （注：部分内容可能由 AI 生成）
