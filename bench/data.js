window.BENCHMARK_DATA = {
  "lastUpdate": 1789285825850,
  "repoUrl": "https://github.com/LoveDaisy/ice_halo_sim",
  "entries": {
    "Single-worker Throughput": [
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "39bb35c6eee19cccafa61c3f38f67c65cbfec9b6",
          "message": "Merge pull request #264 from LoveDaisy/chore/test-premise-expiry-and-gate-justification\n\ntest: retire five dead observation channels and one lying marker",
          "timestamp": "2026-08-11T20:15:43+08:00",
          "tree_id": "d2f97ffb2e4eada1402851069e948df82831d91b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/39bb35c6eee19cccafa61c3f38f67c65cbfec9b6"
        },
        "date": 1786451264380,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 438872.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586461.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 395260.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 354098.7,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "a1957e028279c05871bc50cbd1001cacb1aa2ee8",
          "message": "docs: make a completeness claim carry the same burden of proof as adding code\n\n\"Covers all 20 panels with zero omissions\" reads as an achievement and passes\nreview unchallenged; \"this class is not worth covering\" has to be argued for.\nThat asymmetry is the default state rather than anyone's choice, so completeness\nwins every conflict without a single person advocating for it -- including\nconflicts against the budget the same task committed to.\n\nPR #261 is the measured instance: a pre-committed target of -30% de-commented\ntest lines (baseline 21,336, pinned by two independent counters with 52/52 files\nzero diff) landed at -8.7%, and the coverage backfill demanded by \"20 panels,\nzero omissions\" accounts for roughly a third of the miss. Escape-defect density\nover those same files had already been measured and spans 8x; the equal-weight-\nper-panel split discarded that measurement.\n\nThe rule asks for the justification, not the reduction. Whether a leaner suite\nwould have let more defects escape is a counterfactual and untestable, so this\nis explicitly not a mandate to cut -- only a requirement that an equal-weight\npartition state its reason when a per-member value measure is available.\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>",
          "timestamp": "2026-08-12T08:19:41+08:00",
          "tree_id": "45eab6bd9bc7f45bb51575ce022fbe40ff3e9e32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a1957e028279c05871bc50cbd1001cacb1aa2ee8"
        },
        "date": 1786494687763,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 447247.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 589756.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398049.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 309723.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3b06a512d9e1c500bf53c2ae03ebc98e78a8ee6d",
          "message": "Merge pull request #267 from LoveDaisy/chore/perf-doc-machine-provenance\n\ndocs: 远程验证文档按「角色 / 主机绑定」分层，并写入新参照机 recipe",
          "timestamp": "2026-08-12T15:34:26+08:00",
          "tree_id": "622acaacaf2c20558ea2765a841267f06017335f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3b06a512d9e1c500bf53c2ae03ebc98e78a8ee6d"
        },
        "date": 1786520730034,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 379972.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584669.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 376089.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 390191.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "660cb36f3211d40db009ebe4cb974d819900051d",
          "message": "Merge pull request #268 from LoveDaisy/task/msvc-portability-test-env-helper\n\nfix(test): 把 setenv/unsetenv 的 #ifdef 收敛成一个有名字的 helper",
          "timestamp": "2026-08-12T20:45:55+08:00",
          "tree_id": "a60dffae6ac4ce8bfff90f14b0a1c3d1815c8fe7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/660cb36f3211d40db009ebe4cb974d819900051d"
        },
        "date": 1786539473240,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 589968.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 393704.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 340003,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8684aa9e5d304592bc3fe6a782307b7a9da8c704",
          "message": "Merge pull request #269 from LoveDaisy/task/win-static-crt-cmp0091\n\nfix(build): 让 CMP0091 真正生效，Windows 发布产物链接静态 CRT",
          "timestamp": "2026-08-13T17:00:34+08:00",
          "tree_id": "9e6e68dabd87f16fcb817b3f0ef3dcfef9246ada",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8684aa9e5d304592bc3fe6a782307b7a9da8c704"
        },
        "date": 1786612346240,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 336117.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587389.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 472074.4,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 307294.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7da5553043fc957ef127227fd09c6bd2a434969a",
          "message": "Merge pull request #270 from LoveDaisy/docs/gui-blueprints\n\ndocs(gui): 落盘视觉语言与布局架构两份 GUI 蓝图",
          "timestamp": "2026-08-14T01:14:13+08:00",
          "tree_id": "d80b51f210e34913cdcdae277891c8370b111d51",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7da5553043fc957ef127227fd09c6bd2a434969a"
        },
        "date": 1786641950948,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 389208.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583821.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396528.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 348598.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "de2400d72e0c74815654b5c0798537d321c36342",
          "message": "Merge pull request #271 from LoveDaisy/task/gui-visual-language\n\nfeat(gui): 落地 GUI 视觉语言——单一 owner、比例字体、量化节奏、调色板与语义色",
          "timestamp": "2026-08-14T08:18:19+08:00",
          "tree_id": "e4dae219cff3b357fa69562f4581b9a13c1244c7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de2400d72e0c74815654b5c0798537d321c36342"
        },
        "date": 1786667408354,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 358773.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583079.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 375563.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 342813.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e5a855759f2a1e535cf1ff56144d57a93472b4c1",
          "message": "Merge pull request #272 from LoveDaisy/feat/new-gui-layout\n\nfeat(gui): 新 GUI 布局——「文档 | 图像 | 运行」三区重组（集成分支）",
          "timestamp": "2026-08-18T13:27:49+08:00",
          "tree_id": "4d399707fc655846f8bae9083488c3c91c9ce3aa",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e5a855759f2a1e535cf1ff56144d57a93472b4c1"
        },
        "date": 1787031512594,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 334461.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583724,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 466338.1,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 305972.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a66c523b420650ecb6f8abbe787e38f506ba4e3",
          "message": "Merge pull request #273 from LoveDaisy/feat/gui-form-refinement\n\nfeat(gui): 控件形态精修——宽度 token、PropertyRow 与排版秩序",
          "timestamp": "2026-08-19T02:57:24+08:00",
          "tree_id": "732853fea0235432d76a2465b680be58ccc15e51",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a66c523b420650ecb6f8abbe787e38f506ba4e3"
        },
        "date": 1787080137147,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 362596.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 589562.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 667683.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 294919,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "f1228505c4ed659c006158629ed8b4501eba7074",
          "message": "docs(gui-layout): 记录内测否决，蓝图从待办降为设计记录\n\nv4.4.2（老 shell）与 v4.4.2-new（新 shell）小范围内测对比后，几乎全部\n内测用户选择回到老 shell。main 回退到 PR #271：视觉语言层留下，形态层\n（PR #272 shell 重组 + PR #273 控件精修）退出，实现保存在分支\nfeat/new-gui-layout 与 tag/release v4.4.2-new。\n\n三处改动都是为了让下一个读者不把已被否决的方向当成在途的待办：\n\n1. gui-layout-architecture.md 顶部状态改写 + 新增 §8。记下三件事：反馈\n   粒度未知（聚合结论没区分拒的是形态还是外观，故保留视觉语言层既不由\n   它支持也不被它否定，下一步取证是老 shell 上单发视觉语言层做窄 A/B）；\n   方法层教训（原型验收与 owner 上手两道闸共享同一盲区——都在问「形态\n   本身好不好」，没问「熟练用户是否愿意换」，而后者才是内测在问的）；\n   以及没有被否决的部分（§0 诊断对今天的老 shell 仍为真，§5 六条被推翻\n   形态不恢复候选资格）。\n\n2. gui-visual-language.md 更正时态。该文 §4 定案随 PR #271 留在 main 上，\n   但文中多处把 docking 迁移写成在途的事，回退后不再成立，一律改读作\n   「将来任何一次面板重排」。同时补回 §7 正文字体的收口——字体定案\n   （Roboto Medium 15 构建期嵌入）随 PR #271 落地，而写下这条收口的文档\n   改动落在 PR #273 里，被本次回退一并带走，留下文档说「未定案」而代码\n   已定案的漂移。\n\n3. AGENTS.md 两条索引同步。索引是这两份文档唯一的必经检索入口，否决\n   记录只写在文档里而不写在索引上，等于没写。",
          "timestamp": "2026-08-26T10:15:35+08:00",
          "tree_id": "0508524749ee48fed4c9dbf374750ff51a0579f3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1228505c4ed659c006158629ed8b4501eba7074"
        },
        "date": 1787711358071,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 397295.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586121.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 379053.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "840bc16a46aa7fc6e6265bd34609af330a10af64",
          "message": "docs(gui-layout): 形态层的锚点从 v4.4.2-new 标签改为分支 + commit\n\n内测反馈已到手，v4.4.2-new 标签与 release 随之删除（留着它就是把已被\n否决的界面挂在 Latest release 上发给外部用户）。但 §8 与 AGENTS.md 索引\n都拿这个标签当「形态层保存在哪里」的锚点，标签一删锚点就悬空。\n\n改为锚在分支 feat/new-gui-layout 与 commit 7a66c523——commit hash 是\n永久锚点，分支是可读入口。§8 同时留一句说明标签删除的原因和重出该构建\n的办法（从该分支重新打标签），免得下一个读者以为构建丢了。",
          "timestamp": "2026-08-26T10:28:26+08:00",
          "tree_id": "c793fa450b5d26416c5676defa6dd7ccb9d2badc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/840bc16a46aa7fc6e6265bd34609af330a10af64"
        },
        "date": 1787712077664,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 359688.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586359.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397075.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345279.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "9be4b99e4907e79992121a4e19f6dd742ba03dea",
          "message": "docs(gui-layout): 原型取证锚点随原型分支一同退役\n\n三个 spike 分支（gui-layout-prototype / gui-visual-language / imgui-docking）\n从未推送、只存在于本地，随新布局方向被内测否决一并删除。布局蓝图开头\n把其中 gui-layout-prototype 及三个 commit 写作「取证锚点」，分支一删这行\n就指不到任何东西。\n\n改为如实说明：取证现场已不存在，§1–§5 此后是已记录的判断而非可重新核验\n的断言；要看那一版形态实际长什么样，去 feat/new-gui-layout——同一形态的\n完整实现，完成度高于原型，只不含 §5 那六条从未被实现的候选。\n\n§5 开头补一句界定：六条结论不因原型删除而撤销，重提的一方承担举证责任。\n顺带修一处漂移：视觉语言 §4.1 仍写着「具体字体尚未定案」，而 §7 的收口\n和 main 上的代码都已是 Roboto Medium 15。",
          "timestamp": "2026-08-26T11:34:40+08:00",
          "tree_id": "f4517359a5286de30e0bd6f6f01ac44f5b0b416e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9be4b99e4907e79992121a4e19f6dd742ba03dea"
        },
        "date": 1787715956357,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 375181.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588105.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 484537.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 309385.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "41354876b363944c8882dba3c46014eb797382aa",
          "message": "Merge pull request #274 from LoveDaisy/task/bg-image-filtering\n\nfix(gui): 底图纹理改用 mipmap + trilinear，修缩小显示时的欠采样混叠",
          "timestamp": "2026-08-26T20:00:46+08:00",
          "tree_id": "a5fbb679ee0ffcf9c28c9aec23ea64e6663e04a8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/41354876b363944c8882dba3c46014eb797382aa"
        },
        "date": 1787746335122,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 382839,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586405.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 377604.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 342723.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f17b841b725042bd28408b6db3fd1da5cfc7fa8f",
          "message": "Merge pull request #275 from LoveDaisy/task/bg-image-transform\n\nfeat(gui): 底图可平移缩放，让裁剪过的照片能与仿真结果对齐",
          "timestamp": "2026-08-26T20:20:53+08:00",
          "tree_id": "24754ad01298b2e39006a4bc1d728369562e429c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f17b841b725042bd28408b6db3fd1da5cfc7fa8f"
        },
        "date": 1787747556543,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 355927,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582948.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 486623.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 340238.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e23d669872bbfb1762a95659960d412d793c55e9",
          "message": "Merge pull request #276 from LoveDaisy/task/crystal-enable-toggle\n\nfeat(gui): 晶体卡新增「参与仿真」toggle，替代把权重拖到 0",
          "timestamp": "2026-08-26T20:39:00+08:00",
          "tree_id": "4b0f2abbc5fcc90f68d6ae6281bfef446c82cc03",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e23d669872bbfb1762a95659960d412d793c55e9"
        },
        "date": 1787748626856,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 420703.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587142,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396388.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345168.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "423ff22e33a81fc6521fb8da4f55037e88e7ac93",
          "message": "docs(gui-layout): 内测反馈的细粒度到手，改掉三处已被它推翻的记录\n\n上一轮记录写于反馈只有聚合结论时，有三处现在是错的，且都写在下一个读者\n必经的位置上。\n\n1. 「反馈粒度未知」作废。细粒度反馈是：配色被接受（用户对配色的接受范围\n   很宽），被拒的是形态，理由具体——不如老 plain 布局一眼看到所有信息，\n   典型操作「同时快速调整冰晶与太阳高度」要多点好几步、来回切换不便。\n   于是原计划的「老 shell 上单发视觉语言层做窄 A/B」不必做了：那道取证是\n   为了问出粒度，粒度已经有了。\n\n   机制不是打磨不足，是 master-detail 的结构性代价：老 shell 左栏晶体卡与\n   右栏 Scene（含太阳）永久同时在屏，导航成本为零；新形态里太阳是检视器的\n   一个 page，晶体是同一检视器的另一个 page，一次只看得见一个对象。\n\n2. 方法层教训改写。原先写作「两道闸共享盲区＝没问熟练用户是否愿意换，\n   验收链必须含一条能测迁移成本的证据」——这个说法经不起推敲：本地开发阶段\n   必然只能问「这个形态好不好」，必然要发版才拿得到用户反馈，那不是一道\n   本可设而没设的闸，而是结构性事实；发版、拿反馈、便宜回退、分支留存，\n   这个环当时是通的。真正的偏差是验收问错了量——两道闸问的都是「形态本身\n   好不好用」，而用户答的是两个本地就能机械量出来却从没被量过的数：常见\n   任务的操作步数，以及一屏同时可见的字段集合。\n\n3. §0 第一条补一个维度。§0 骂老 shell「分割轴任意」属实，但「任意」不等于\n   「差」：那条任意的轴恰好让最常一起调的两组永久同屏，而 §0 从未度量过\n   同屏可见性这一维，用户却只在这一维上表了态。因此追加一条硬约束——将来\n   任何一次重排，最常一起调的字段组必须保持同屏可见——地位等同 §5 那六条\n   被推翻形态。\n\ngui-visual-language.md 顶部同步：那条「不构成外观已被接受的证据」作废，但\n边界要写清楚，被问到的只有配色，§4 其余条目仍只是「没有被反对」。\nAGENTS.md 两条索引一并同步——索引是这两份文档唯一的必经检索入口。\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>",
          "timestamp": "2026-08-26T20:49:35+08:00",
          "tree_id": "6b66463dd3987722c0987a1f07a416d5332cca1b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/423ff22e33a81fc6521fb8da4f55037e88e7ac93"
        },
        "date": 1787749579976,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 349237,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584790.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 377980.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 392668,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "eea0a7b0268762f9590f8d50912a3662a80e6936",
          "message": "Merge pull request #277 from LoveDaisy/task/gui-overlay-table\n\nfeat(gui): Overlay 辅助线组改为 6 列表格形态",
          "timestamp": "2026-08-29T12:19:58+08:00",
          "tree_id": "7e912e59f4173ec789186af3f24b4e7329ddef8d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/eea0a7b0268762f9590f8d50912a3662a80e6936"
        },
        "date": 1787977865951,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 450769.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583003.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 437407.7,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 312610.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ff02ed3a487ac6e46dd2720351057ed2314aa60b",
          "message": "Merge pull request #278 from LoveDaisy/task/full-sphere-roll-flip\n\nfix(core+gui): 全球面快路径补 roll 旋转对称条件，修滑条端点浮点漂移导致的采样路静默切换",
          "timestamp": "2026-08-29T13:20:57+08:00",
          "tree_id": "12a2380afc7b07d8b15f664fa3f2bf288d3e7f1e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ff02ed3a487ac6e46dd2720351057ed2314aa60b"
        },
        "date": 1787981566213,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 369068,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 579860.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 487148.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 344830.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6320bbab9d388df3b07a528374e61df82d920a6b",
          "message": "Merge pull request #279 from LoveDaisy/task/gui-theme-color-closure\n\nrefactor(gui): 颜色收口——色槽补齐 58/58 + 调用点裸字面量逐处 disposition",
          "timestamp": "2026-08-29T15:06:01+08:00",
          "tree_id": "082d1f2e575f42837f85a5f180ed8d57c1ca7176",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6320bbab9d388df3b07a528374e61df82d920a6b"
        },
        "date": 1787987821445,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 323186.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582739.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 381507.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 299533.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "994823ec3a42f7c5fb27150249b58b30ac4f6336",
          "message": "Merge pull request #280 from LoveDaisy/task/overlay-table-acceptance-fixes\n\nfix(gui): Overlay 表格人工验收三条修复",
          "timestamp": "2026-08-29T22:17:55+08:00",
          "tree_id": "6d0628441baf2aabfb41489d69d226d3905d11b0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/994823ec3a42f7c5fb27150249b58b30ac4f6336"
        },
        "date": 1788013775673,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 393564.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586494.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 515912.3,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 341710.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "26e730907f491d40869033a4ce67ac66edc88f8a",
          "message": "Merge pull request #281 from LoveDaisy/task/gui-label-column-gap-alignment\n\nfix(gui): 行末标签列左缘对齐 + 间距收敛为单一 owner",
          "timestamp": "2026-08-30T11:28:21+08:00",
          "tree_id": "51266763b2f5e9a7423d99f0a75aaf7d05257f11",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/26e730907f491d40869033a4ce67ac66edc88f8a"
        },
        "date": 1788061020777,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 442084.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 578613.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 514765.8,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 415212.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "37658798751c385f405d25527fc3226fb807ff08",
          "message": "Merge pull request #282 from LoveDaisy/task/gui-entry-card-layout-and-crystal-identity\n\n晶体卡片 layout 重排 + 晶体身份可寻址 + Colors 面板编号/失效态",
          "timestamp": "2026-08-30T13:34:44+08:00",
          "tree_id": "23118e5367b29ce0c82bda8278360f565c79c8b8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37658798751c385f405d25527fc3226fb807ff08"
        },
        "date": 1788068771832,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 460712.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587210.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 391429.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 343581.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "d63befda26e1912dcf6d6d8cc67efd7f34eb7ac2",
          "message": "bump patch version for release",
          "timestamp": "2026-08-30T17:17:40+08:00",
          "tree_id": "ae2567feaaad662c625ed888e28482837014d4c8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d63befda26e1912dcf6d6d8cc67efd7f34eb7ac2"
        },
        "date": 1788082217533,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 382429.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 580997,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 489287.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 408901.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "53299d835046ba74d6f2897c4a1566993368a6bf",
          "message": "Merge pull request #283 from LoveDaisy/task/gui-fisheye-lens-border\n\nGUI: 鱼眼镜头有效区边框辅助线",
          "timestamp": "2026-08-30T22:06:58+08:00",
          "tree_id": "2ba992405cad4908ee5cc797c9404a38a110ff48",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/53299d835046ba74d6f2897c4a1566993368a6bf"
        },
        "date": 1788099491231,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 366626.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585641.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396203.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 347464.8,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "68f3be6a4fc6353c202b92f6538e7b9c3d5d1100",
          "message": "Merge pull request #284 from LoveDaisy/task/retire-comma-raypath-separator\n\n退役 raypath 逗号连接符：静默算错改为指名改法的拒绝 + 加载期迁移",
          "timestamp": "2026-08-30T22:29:34+08:00",
          "tree_id": "e9cc435a031fe70e00ae4ab2e1c43c075f39eb24",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/68f3be6a4fc6353c202b92f6538e7b9c3d5d1100"
        },
        "date": 1788100786901,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 471820.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584118.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 375325.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 366955.1,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c798112109aa5fb9e2f85564fd0f9bfb59e0259",
          "message": "Merge pull request #285 from LoveDaisy/task/user-defaults-schema-version\n\n给 user_defaults.json 盖上独立的 schema 版本戳（只记录，不设闸，不迁移）",
          "timestamp": "2026-08-31T08:46:01+08:00",
          "tree_id": "c6063d0d04375864505dbcfd05c8f5eba511d092",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6c798112109aa5fb9e2f85564fd0f9bfb59e0259"
        },
        "date": 1788137853998,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 382685,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584735.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 401949,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 352620.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "034ab22193bfe8cf4e9efb4aac1599e82c5e3308",
          "message": "Merge pull request #286 from LoveDaisy/feat/adjustable-background-color\n\n可调背景颜色：GUI/CLI 五路一致 + core 定义域掩码 + 注记层处置",
          "timestamp": "2026-08-31T12:48:11+08:00",
          "tree_id": "465ab3c2e3bc53104f4df724fba2c95cd70a2d2c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/034ab22193bfe8cf4e9efb4aac1599e82c5e3308"
        },
        "date": 1788152403861,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 374559.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583777,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 430017.4,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 334834.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8054154bedaaf1ce926cd9c7d44a6c1c548cc115",
          "message": "Merge pull request #287 from LoveDaisy/feat/absolute-ev\n\nfeat: 绝对 EV —— cross-simulation 可比的曝光尺度",
          "timestamp": "2026-08-31T15:03:03+08:00",
          "tree_id": "e66dfd46e67488974ac2ab7f738dc49b2e56328a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8054154bedaaf1ce926cd9c7d44a6c1c548cc115"
        },
        "date": 1788160548231,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 340411,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585955.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 600361.4,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 343529.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5360f028303b6963e69eb26ba999c203a3f3018e",
          "message": "Merge pull request #288 from LoveDaisy/feat/cli-gui-render-parity\n\n让导出的 config 诚实描述用户所见 + 建 CLI↔GUI 出图对照闸",
          "timestamp": "2026-09-01T10:20:29+08:00",
          "tree_id": "1d4a30db33e4386ab9a58ed55adc7521892bc4bb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5360f028303b6963e69eb26ba999c203a3f3018e"
        },
        "date": 1788229995879,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 326479.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 580948,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400577.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 344570.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "93c163fe3b8be8017bff712da303b6afa9ba8c03",
          "message": "Merge pull request #289 from LoveDaisy/task/lens-json-names-oob\n\nfix(gui): 修 kLensTypeJsonNames 越界读（用户可达崩溃）",
          "timestamp": "2026-09-01T11:38:10+08:00",
          "tree_id": "386dbbee06c0b18a0469f3dd58b480cc1f8c7e05",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/93c163fe3b8be8017bff712da303b6afa9ba8c03"
        },
        "date": 1788234592767,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 437722.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586103.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 392229.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 318645.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9faf145b0edde6af078573ca8726d7ce246411df",
          "message": "Merge pull request #290 from LoveDaisy/task/preview-solid-angle-jacobian\n\nfeat(gui): 预览 shader 补上目标镜头的相对照度，使非等面积投影下 GUI 与 CLI 可逐像素比",
          "timestamp": "2026-09-01T13:00:21+08:00",
          "tree_id": "89c8de38143255c6034f17752c7461109dd74f23",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9faf145b0edde6af078573ca8726d7ce246411df"
        },
        "date": 1788239563097,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 420566.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582659,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 484119.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 344268,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "03e21b4dd2baadc1325958ed61d1293e3d6434c9",
          "message": "Merge pull request #291 from LoveDaisy/feat/fisheye-domain-widening\n\nfeat(core): 单镜头鱼眼定义域按 lens 放宽到 θ≤180，与 GUI 对齐",
          "timestamp": "2026-09-01T15:17:01+08:00",
          "tree_id": "db0ee29334ec16918d95c3230818b73c6aa64f80",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/03e21b4dd2baadc1325958ed61d1293e3d6434c9"
        },
        "date": 1788247623405,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 409874.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586827.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 519904.3,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 410290.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a6034051812988c3e1fb7639296b5d3fbdbf8375",
          "message": "Merge pull request #292 from LoveDaisy/feat/core-annotation-layer\n\nfeat(core): 注解层补齐——辅助线与文字 label 收敛为 core 单一来源",
          "timestamp": "2026-09-02T04:45:19+08:00",
          "tree_id": "fe59f2f65430792483491f011bdddca70953ff32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a6034051812988c3e1fb7639296b5d3fbdbf8375"
        },
        "date": 1788296267506,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 392552.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586047.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 386742.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 316035.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fc5de377009120dd26063703a16f31027aaffd62",
          "message": "Merge pull request #293 from LoveDaisy/feat/test-time-and-scope-discipline\n\nfeat(ci/doc): 测试时间预算的 owner —— 实测拓扑、分片重装箱、分层契约",
          "timestamp": "2026-09-02T09:04:32+08:00",
          "tree_id": "cfe6beb5c25bbb4014f67bf142635e635a347f4e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc5de377009120dd26063703a16f31027aaffd62"
        },
        "date": 1788311802181,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 430483,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587050.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 399588.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 365676.8,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "32515f9970bfd614540c36dece2c55fe71eee1a6",
          "message": "Merge pull request #294 from LoveDaisy/chore/annotation-doc-and-diagnostics-gaps\n\nchore(doc/cli): 补 zenith_nadir schema 文档；renderer 超限诊断指向真正的上限",
          "timestamp": "2026-09-02T13:14:20+08:00",
          "tree_id": "377cc3a2fbdf13228f8c678cf3391e0e9631022c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/32515f9970bfd614540c36dece2c55fe71eee1a6"
        },
        "date": 1788326817910,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 388276.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582699.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397483.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 348484.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "18624004d00f891498779bf12248f36046859a41",
          "message": "Merge pull request #295 from LoveDaisy/feat/fast-e2e-dominant-test\n\ntest(e2e): smoke 按 config 拆成独立 pytest item —— 收集粒度对齐调度粒度，零覆盖损失",
          "timestamp": "2026-09-02T14:42:18+08:00",
          "tree_id": "c633f9361f8df5a4557dd160ab557794d6e71044",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/18624004d00f891498779bf12248f36046859a41"
        },
        "date": 1788332011110,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 339374.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582822.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 449713.2,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1ac3ab63b1d72eb8034782f0b8b8f09e100b4636",
          "message": "Merge pull request #296 from LoveDaisy/task/save-open-visual-consistency-red\n\nfix(gui): .lmc 与 composite 纹理改存纯辐亮度，渐晕由显示端统一补上",
          "timestamp": "2026-09-02T18:00:52+08:00",
          "tree_id": "b7bbed427787b088d61b2fa2e14ac122e1150a1d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1ac3ab63b1d72eb8034782f0b8b8f09e100b4636"
        },
        "date": 1788344049697,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 364593.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583581.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 437245,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345181.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "08aa283330101f2ed499cab2ab657abe4bcbb2b2",
          "message": "Merge pull request #297 from LoveDaisy/feat/lens-projection-semantics\n\nfeat(core): 收口 ProjectExitToPixel 遗留的三条 core↔GUI 分歧（参考图只重拍一次）",
          "timestamp": "2026-09-02T23:11:02+08:00",
          "tree_id": "db3c8f71967ab4435d5b915e7c9047561643c424",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/08aa283330101f2ed499cab2ab657abe4bcbb2b2"
        },
        "date": 1788362661945,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 452259.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582599.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 683527.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 346856,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dfe34bc13b012f367d913460738f8dc02a50faa0",
          "message": "Merge pull request #298 from LoveDaisy/chore/gui-unit-heartbeat-wallclock-margin\n\ntest(gui-unit): 心跳用例改 wait-until，墙钟余量 250ms → 秒级",
          "timestamp": "2026-09-03T01:12:40+08:00",
          "tree_id": "1b9799920e428da55ee87172d76c0d6689c879f4",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfe34bc13b012f367d913460738f8dc02a50faa0"
        },
        "date": 1788369958664,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 447075.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585220.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 488611.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 344775,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "82310d302119a07fff51338e811ffd189ee0aac1",
          "message": "Merge pull request #299 from LoveDaisy/feat/relative-ev-anchor\n\nfeat(core): 把 relative 曝光锚点做对 —— 锚到固定全天缓冲，CLI 与 GUI 消费同一个数",
          "timestamp": "2026-09-03T15:07:51+08:00",
          "tree_id": "582e122487ca1d78c26a6dae0bb7dc213f067af6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/82310d302119a07fff51338e811ffd189ee0aac1"
        },
        "date": 1788420073654,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 381836.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584684.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 488166.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 348213.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c3c5042e4f17c60ecba3cbf6ff38267594826fa",
          "message": "Merge pull request #300 from LoveDaisy/fix/scene-cnt-publish-ordering\n\nfix(server): 记账先于发布，消除批次静默丢失的竞态窗口",
          "timestamp": "2026-09-03T19:19:00+08:00",
          "tree_id": "a4a02453a42967340bf873f3b689410b89d032b9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6c3c5042e4f17c60ecba3cbf6ff38267594826fa"
        },
        "date": 1788435084338,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 368794.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583443.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 393386.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321914.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "365ba58f71d4354ec1035328c12ffd3427c6492e",
          "message": "Merge pull request #301 from LoveDaisy/feat/crystal-slider-domain-and-format\n\nfeat(gui): 柱晶高度域扩到 1e-4，滑杆显示格式由映射闭式定下界并在编译期判错",
          "timestamp": "2026-09-04T15:31:05+08:00",
          "tree_id": "c649e8f7c8eff961a4f64eb3b5956f027157ad32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/365ba58f71d4354ec1035328c12ffd3427c6492e"
        },
        "date": 1788507854909,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 399172.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 580539.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 375039.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 355399.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ddc94c02faa23de01ef59b90d25f106843699744",
          "message": "Merge pull request #302 from LoveDaisy/fix/preview-source-gather-half-texel\n\nfix(gui): 预览 shader 从 dual-EA 源纹理取样时多出的半个纹素",
          "timestamp": "2026-09-04T18:44:53+08:00",
          "tree_id": "5bbccc2a028e253d4dade882c65deb804b594261",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ddc94c02faa23de01ef59b90d25f106843699744"
        },
        "date": 1788519439974,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 380152.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585938.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398646,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 317856.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7381ca162338ea5105412cc97719e7c7d7eeb1cb",
          "message": "Merge pull request #303 from LoveDaisy/feat/alloc-churn-and-rng-bounds\n\nperf(core): all_data 缓冲区复用 + GetUniform 下标边界收敛到单一 owner",
          "timestamp": "2026-09-04T20:16:18+08:00",
          "tree_id": "f4f4cdc4118328fb3d085e6dd57776fb9c3f53b7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7381ca162338ea5105412cc97719e7c7d7eeb1cb"
        },
        "date": 1788524960910,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 394049.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 606435.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432207.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 380512.4,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1271f2699d6339b658ee96d3cf0502055c2b034a",
          "message": "Merge pull request #304 from LoveDaisy/fix/gui-preview-single-render-path\n\nfix(gui): 预览与 Screenshot 导出收敛到单一离屏 FBO 渲染路径",
          "timestamp": "2026-09-04T20:53:45+08:00",
          "tree_id": "1ea3fd858b1979513692e7dd728217e33f1635d7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1271f2699d6339b658ee96d3cf0502055c2b034a"
        },
        "date": 1788527137952,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 374725.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 607559.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 429598,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61328fbedb7c6acfaf163c040563f24285d52ff2",
          "message": "Merge pull request #305 from LoveDaisy/feat/sky-reference-points\n\nfeat(gui,core): 天空参考点标记六点泛化 + Look At 视角预设，共享 core 单源方向表",
          "timestamp": "2026-09-05T02:15:06+08:00",
          "tree_id": "66f5811bd94711b21e96cefdfa28e8d5b7dcf841",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61328fbedb7c6acfaf163c040563f24285d52ff2"
        },
        "date": 1788546554777,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 431101.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609374,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 394675.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 372539.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9327bb7a34e0095c493f6408acc9067e2512fd61",
          "message": "Merge pull request #307 from LoveDaisy/fix/gui-test-harness-gates\n\ntest(gui): 闭合 gui_test 现场三处守卫可信度缺口——恒红闸 / 缺失的互比闸 / 注释纪律一致性",
          "timestamp": "2026-09-05T14:10:18+08:00",
          "tree_id": "2ca052b6512de4cebc819bec0838eb5723d8a739",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9327bb7a34e0095c493f6408acc9067e2512fd61"
        },
        "date": 1788589259981,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 385001.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 606738.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 438754.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "1f3a9c864ad290679f4f638c62e9bbf6be03a855",
          "message": "Merge pull request #307 from LoveDaisy/fix/gui-test-harness-gates\n\ntest(gui): 闭合 gui_test 现场三处守卫可信度缺口——恒红闸 / 缺失的互比闸 / 注释纪律一致性",
          "timestamp": "2026-09-05T14:15:53+08:00",
          "tree_id": "2ca052b6512de4cebc819bec0838eb5723d8a739",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1f3a9c864ad290679f4f638c62e9bbf6be03a855"
        },
        "date": 1788590057637,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 424857.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608301,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 440448.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 323562.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fa044a6142525e887ef3e31b89027a953f00343b",
          "message": "Merge pull request #308 from LoveDaisy/fix/clamp-uniform-zero-n\n\nfix(core): ClampUniformToIndex 的 n==0 前提由 assert 改为 FatalAbort 硬守卫",
          "timestamp": "2026-09-05T14:40:02+08:00",
          "tree_id": "b9b2c7baef1b585fa751319ac718c1a01132d770",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa044a6142525e887ef3e31b89027a953f00343b"
        },
        "date": 1788591146989,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 352808.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609574.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 431369.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 343380.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "92c4036ebd9dc10187397db16e7a435ba69af19b",
          "message": "Merge pull request #309 from LoveDaisy/chore/native-arch-measurement-hygiene\n\nfeat(bench): 让「本地构建不是出货二进制」这件事在取数字的地方可见（ISA 出处）",
          "timestamp": "2026-09-05T14:54:12+08:00",
          "tree_id": "d6e704e339bd4d14891a0549aafe20bd3799c40e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/92c4036ebd9dc10187397db16e7a435ba69af19b"
        },
        "date": 1788592052582,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 408206.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609052.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432363.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 352473.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5aea061e953c620bfe6709bf6f2ce87103402f05",
          "message": "Merge pull request #310 from LoveDaisy/fix/cli-label-viewport-clamp\n\nfix(cli): overlay label 视口 clamp 收敛为 GUI/CLI 共享的单一实现",
          "timestamp": "2026-09-05T15:09:23+08:00",
          "tree_id": "f073e0e74a8ffe5cbb3e2967c3330a884cb4279c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5aea061e953c620bfe6709bf6f2ce87103402f05"
        },
        "date": 1788592954874,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 441332.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608037,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 434390.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373513.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f1822e52c6d59ab2c203aa7ef388cb2b50f4e2df",
          "message": "Merge pull request #311 from LoveDaisy/feat/gpu-backend-user-preference\n\nfeat(gui): \"Use GPU\" 可存为个人默认（覆盖文件新增 app 根键），工厂默认仍是 CPU",
          "timestamp": "2026-09-05T15:44:47+08:00",
          "tree_id": "b61f1340307f90b27cb42c9f0d3c8a930fc8a63c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1822e52c6d59ab2c203aa7ef388cb2b50f4e2df"
        },
        "date": 1788594953269,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 347363.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608835.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 707823.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 429647.9,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5015321af574f4bc5efa1787fa3feefc87a835b",
          "message": "Merge pull request #312 from LoveDaisy/fix/benchmark-steady-window-outlier\n\nfix(bench): active_short 不得拿 IDLE 检测延迟当分母（14-29x 野值的根因）",
          "timestamp": "2026-09-05T15:59:52+08:00",
          "tree_id": "e502d8ac59d36d9acdffade736d7bf32b619fa08",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5015321af574f4bc5efa1787fa3feefc87a835b"
        },
        "date": 1788595939395,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 489367.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 605436.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 435693.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fdf903868b533c085c845322ee50722158e3a089",
          "message": "Merge pull request #313 from LoveDaisy/scrum/sentinel-liveness-audit\n\ntest(sentinel): retire the one regression sentinel that outlived its mechanism",
          "timestamp": "2026-09-06T00:54:43+08:00",
          "tree_id": "58522174f803a4d43425b4cd8e96c32dbab08dee",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fdf903868b533c085c845322ee50722158e3a089"
        },
        "date": 1788628071388,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 605657.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 394293.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 371888.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "aa3ec3edc826ec5ba4b7cb59fb1e102222309889",
          "message": "Merge pull request #314 from LoveDaisy/scrum/worker-and-batch-granularity\n\nperf(cpu): cap the automatic worker count at 10, expose --workers, and fix a hit-loop buffer overflow",
          "timestamp": "2026-09-06T10:48:35+08:00",
          "tree_id": "0f52437edfb35a55cb432bf05c2ba64797af8a5f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/aa3ec3edc826ec5ba4b7cb59fb1e102222309889"
        },
        "date": 1788663718681,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 481881.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 612053.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 542202,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 363369.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5cb81a9565ff0e1394a957d97ad2078d0d8f9310",
          "message": "Merge pull request #315 from LoveDaisy/scrum/changelog-backfill-and-release-notes\n\ndocs(release): 回填 v4.1.4 起 31 个版本的 CHANGELOG，并把它接进发版链路",
          "timestamp": "2026-09-06T16:24:48+08:00",
          "tree_id": "079d5a016ac9ac51339233ba0779369c19e64745",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5cb81a9565ff0e1394a957d97ad2078d0d8f9310"
        },
        "date": 1788683893064,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 366177.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610424.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 449994.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 372612.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c04ad137d42e4a3ed51cd137b75d2d97092b7864",
          "message": "Merge pull request #316 from LoveDaisy/fix/msvc-string-literal-limit\n\nfix(gui,ci): 拆开超 MSVC 上限的 shader 字面量 + 立静态门禁 + CI 触发去重",
          "timestamp": "2026-09-06T23:22:44+08:00",
          "tree_id": "6cd1e5d94251d724205eb0763696031080e86c9d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c04ad137d42e4a3ed51cd137b75d2d97092b7864"
        },
        "date": 1788708803885,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 357379.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610991.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 440025.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 540653.7,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "349656ac94b6b2fd27ecbf4ed35812bea7646a2a",
          "message": "Merge pull request #317 from LoveDaisy/ci/windows-release-image-unify\n\nci: build Windows on the image we actually release from",
          "timestamp": "2026-09-07T01:04:20+08:00",
          "tree_id": "d7e945127aac502e4a16adeca4de66c2fca33ff5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/349656ac94b6b2fd27ecbf4ed35812bea7646a2a"
        },
        "date": 1788714897840,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 442237.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611721,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 438189.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 339521.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "59208e341a7a63e1f22366ec0fbc47211fd93950",
          "message": "Merge pull request #318 from LoveDaisy/test/e2e-cost-and-oracle-audit\n\ntest(e2e): 按「每个测试为自己的开销举证」审计套件成本，恢复预算余量",
          "timestamp": "2026-09-07T04:18:31+08:00",
          "tree_id": "a180e65114ec4dbebe9febd562ebcb9d7dcb6dcd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/59208e341a7a63e1f22366ec0fbc47211fd93950"
        },
        "date": 1788726574327,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 454985.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610617,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 508564.3,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 374070.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e707b15d31a6676c8f4147a0b0cfe62dfc452995",
          "message": "Merge pull request #319 from LoveDaisy/fix/gui-entry-delete-vs-open-editor\n\nfix(gui): keep the edit modal bound to its entry across a delete",
          "timestamp": "2026-09-08T11:14:03+08:00",
          "tree_id": "493abe69703c95a59432e4a6f6623947ade2f407",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e707b15d31a6676c8f4147a0b0cfe62dfc452995"
        },
        "date": 1788837905475,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 366604,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610382.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 393819.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 339860.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "da9e5533acc8c01c61877b6613ddf40bdce9a8b4",
          "message": "Merge pull request #320 from LoveDaisy/fix/cuda-zero-ray-batch-poisons-backend\n\nfix(cuda): stop a zero-ray layer from poisoning the CUDA backend",
          "timestamp": "2026-09-08T17:13:37+08:00",
          "tree_id": "915905aba2ebcbe1abe8eebe327b96458d905c22",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/da9e5533acc8c01c61877b6613ddf40bdce9a8b4"
        },
        "date": 1788859460604,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 431169.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608896.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 436299.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 335202.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d66985dfc19d7e0a2ad278bfb29e8adec87f3adc",
          "message": "Merge pull request #322 from LoveDaisy/test/random-source-exact-assertion-audit\n\ntest: audit random sources behind exact assertions, and refill the lost closed-form fuzz",
          "timestamp": "2026-09-08T19:05:43+08:00",
          "tree_id": "559a5d866b5f56b5751d9d1be56655c2412ea1a2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d66985dfc19d7e0a2ad278bfb29e8adec87f3adc"
        },
        "date": 1788866199114,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 380491.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611147.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 433950.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 436283.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5d738bc07fade93832384583fd5655dda496ae4",
          "message": "Merge pull request #321 from LoveDaisy/ci/organization-and-windows-testing\n\nci(windows): route MSVC compilation through sccache",
          "timestamp": "2026-09-08T20:34:51+08:00",
          "tree_id": "8cbe8c8f0a06164c7e6f448e8e1b9b8f2b5aeff3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5d738bc07fade93832384583fd5655dda496ae4"
        },
        "date": 1788871599160,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 361674,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611795.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 425085.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 428972,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5dab8ac5279acc30e74281d28ec9852f7091260f",
          "message": "Merge pull request #323 from LoveDaisy/feat/annotation-label-line-independence\n\nfeat(config): give the three grid families a line switch of their own",
          "timestamp": "2026-09-08T21:51:50+08:00",
          "tree_id": "17e102e9476ace17c070d7edc5d9863355c01883",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5dab8ac5279acc30e74281d28ec9852f7091260f"
        },
        "date": 1788876183459,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 363514,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610430.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 395620.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 340964.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e06d6f8ee003f53159f4265e0a60478c3912298f",
          "message": "Merge pull request #324 from LoveDaisy/perf/cli-render-poll-floor\n\nperf(cli): poll completion before sleeping, so a render is not floored at 1s",
          "timestamp": "2026-09-08T22:52:18+08:00",
          "tree_id": "0e0c25fa9ce2a27fc0554ad989a39b2c64786891",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e06d6f8ee003f53159f4265e0a60478c3912298f"
        },
        "date": 1788879778701,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 434610.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609291.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 438619.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 361601,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dfb3f72cde1303813ed87b0403da0bf2d5264b86",
          "message": "Merge pull request #325 from LoveDaisy/fix/user-run-vs-backpressure-gate\n\nfix(gui): exempt a user-initiated Run from the commit backpressure gate",
          "timestamp": "2026-09-08T23:07:48+08:00",
          "tree_id": "3e55872359ee24f0c6224f4e627e5dd6964a1d7d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfb3f72cde1303813ed87b0403da0bf2d5264b86"
        },
        "date": 1788880723618,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 393982.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609608.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 435199.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337894.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61afdda60a81f3800fe9d39d0f1790efbed2eb82",
          "message": "Merge pull request #326 from LoveDaisy/ci/cuda-test-tu-compile-coverage\n\nci: compile the CUDA test TUs (close the CUDA×BUILD_TEST empty intersection)",
          "timestamp": "2026-09-09T09:01:05+08:00",
          "tree_id": "d8015a4a11ce1b395d085be4867bade1cd75f056",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61afdda60a81f3800fe9d39d0f1790efbed2eb82"
        },
        "date": 1788916295931,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 398191.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611456.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 439095.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 432172.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5d04ed2b510cf8da1331d69d8a0ec9d064e8688a",
          "message": "Merge pull request #328 from LoveDaisy/feat/gui-import-capability-boundary\n\nfeat(gui): warn on intentionally unsupported capabilities when importing core/CLI configs",
          "timestamp": "2026-09-09T11:28:47+08:00",
          "tree_id": "5fb7bd048fbeeb0720b37f54dc406c24b4f696f7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5d04ed2b510cf8da1331d69d8a0ec9d064e8688a"
        },
        "date": 1788925157511,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 392728,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610925.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 437873.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 362684.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fe4c0778ad67356b7107c49b9f2d1758751cbab0",
          "message": "Merge pull request #329 from LoveDaisy/fix/raypath-load-path-syntax-gate\n\nfix(gui): reject malformed raypath summand rows on the .lmc load path",
          "timestamp": "2026-09-09T11:50:12+08:00",
          "tree_id": "85ae4abe8324f97a506977a0b6b5e2f1ee1a6194",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fe4c0778ad67356b7107c49b9f2d1758751cbab0"
        },
        "date": 1788926440664,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 456080.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611254.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 436450.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 430115.9,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61199dc71bb8afa824f182ecb361c1765212e2ba",
          "message": "Merge pull request #330 from LoveDaisy/build/cpm-cache-shared-default\n\nbuild(cpm): default the dependency-source cache to a machine-level directory",
          "timestamp": "2026-09-09T12:28:59+08:00",
          "tree_id": "401ce6afc69f30c5242ddcb0e9b1c84c265275ad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61199dc71bb8afa824f182ecb361c1765212e2ba"
        },
        "date": 1788928782668,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 372360.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611889.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 437338.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 434947.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "52b769f8831d3826607f24929ab7130e9dc62d1e",
          "message": "Merge pull request #331 from LoveDaisy/refactor/field-set-sentinel-proxy\n\nrefactor(config): guard RenderConfig's field set by member count, not sizeof",
          "timestamp": "2026-09-09T13:13:40+08:00",
          "tree_id": "ea00f972f5073c2d5a34b75fbe200d19d12f07b6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/52b769f8831d3826607f24929ab7130e9dc62d1e"
        },
        "date": 1788931507521,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 457275.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610916,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 439743.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 377284,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "96d644a21248a0968f4866679a3de372c1610833",
          "message": "Merge pull request #334 from LoveDaisy/feat/bg-image-color-picker\n\nfeat(gui): sample Sky Color off the background photo with an eyedropper",
          "timestamp": "2026-09-10T01:16:27+08:00",
          "tree_id": "dacea434bc3720c060ce99ea1e1f7083321478c0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/96d644a21248a0968f4866679a3de372c1610833"
        },
        "date": 1788974899057,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 409623.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610248.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 428039.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 439436.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "258d9d34fde41b6c3c29d2818e91a4a20c13f2af",
          "message": "Merge pull request #335 from LoveDaisy/ci/drop-unused-vendor-apt-source\n\nci: stop depending on a vendor apt source nothing here installs from",
          "timestamp": "2026-09-10T02:19:42+08:00",
          "tree_id": "0ec8ef1453363cf2a4e9bac9403be312d6a3d461",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/258d9d34fde41b6c3c29d2818e91a4a20c13f2af"
        },
        "date": 1788978629071,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 360916.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611670.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 439932.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 364142.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "712eb886076cecd28b4dedc683f8255351558cb5",
          "message": "Merge pull request #333 from LoveDaisy/ci/cache-budget\n\nci(cache): budget the actions/cache quota — fix three prefix-shadowed keys, add ccache to the critical-path leg",
          "timestamp": "2026-09-10T02:35:58+08:00",
          "tree_id": "c53075d893ac829084799f406bdd8c280a195292",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/712eb886076cecd28b4dedc683f8255351558cb5"
        },
        "date": 1788979613946,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 473801.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611061.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 483844.3,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 340987,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "491b117b9a07cdf85de8099529f7811e686abf1e",
          "message": "Merge pull request #336 from LoveDaisy/feat/miller-index-and-wedge-presets\n\nfix(gui,core): give the Miller-index wedge conversion one owner, and correct the presets it was never checked against",
          "timestamp": "2026-09-10T04:26:14+08:00",
          "tree_id": "4e290141061128a452482994544759c4c4475a08",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/491b117b9a07cdf85de8099529f7811e686abf1e"
        },
        "date": 1788986309576,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 413207.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609913.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 421022.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 369755,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a4c526050e74287deacd4473631926deabf13a9",
          "message": "Merge pull request #337 from LoveDaisy/feat/print-mode-subtractive-ink\n\nfeat(render,gui): add a print tone that lays ink on paper instead of adding light to sky",
          "timestamp": "2026-09-10T09:06:10+08:00",
          "tree_id": "7d1fa0a3596ea0279c8d776fa7d4f0baaa8feab6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a4c526050e74287deacd4473631926deabf13a9"
        },
        "date": 1789003109399,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 364368.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610311,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 433708.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 366244,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ae46f7c283910d4fbb4e7a0f2949aef02e5f87df",
          "message": "Merge pull request #338 from LoveDaisy/feat/gui-display-rendering-regroup\n\nfix(gui): regroup the Display Rendering rows and pair the ground swatch with the mode",
          "timestamp": "2026-09-10T14:08:18+08:00",
          "tree_id": "eeaec146eaa0226974bb83e95e4e1b36f34970a9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ae46f7c283910d4fbb4e7a0f2949aef02e5f87df"
        },
        "date": 1789021100736,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 362919.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611994.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 433174,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 368764.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3ad57411dc16f516b6785967efaba5266c88e7b8",
          "message": "Merge pull request #339 from LoveDaisy/feat/test-capi-lib\n\ntest: liblumice_testapi, a test-only export surface beside the product C API",
          "timestamp": "2026-09-10T16:59:47+08:00",
          "tree_id": "91bfa648700adc1c02137e2bab552ca4271f3417",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3ad57411dc16f516b6785967efaba5266c88e7b8"
        },
        "date": 1789031371463,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 333294.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611939.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 426046.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373294.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "724fa7cff5ff0cc832f33d45b08e1bf3d4536f40",
          "message": "Merge pull request #342 from LoveDaisy/feat/annotation-lines-shader-anchors-api\n\ngui: auxiliary lines track the camera every frame again; anchors-only annotation API (v4.28)",
          "timestamp": "2026-09-10T17:18:09+08:00",
          "tree_id": "7f9537581734b6e612b1d500271e44df92102186",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/724fa7cff5ff0cc832f33d45b08e1bf3d4536f40"
        },
        "date": 1789032507905,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 285739.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609018.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 391682.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 447223.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d57f132dc2f546ad001a215fb28d6c917bddabf4",
          "message": "Merge pull request #340 from LoveDaisy/docs/working-discipline-hardening\n\ndocs+hooks: harden two working-discipline rules into criteria and a commit gate",
          "timestamp": "2026-09-10T18:06:39+08:00",
          "tree_id": "1e03b6ab5fd5688bd565295581883fc4c9690d85",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d57f132dc2f546ad001a215fb28d6c917bddabf4"
        },
        "date": 1789035351539,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 344873.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 605823.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 434453.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 348984.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "37b141504a40cc0be937f0d5bf071fef24759171",
          "message": "Merge pull request #341 from LoveDaisy/test/defaults-panel-refs-reshoot\n\ntest(gui): pin the wedge add row in every preset scene, and re-shoot the two that were not",
          "timestamp": "2026-09-10T18:49:54+08:00",
          "tree_id": "42fe951d730ad0cd31b0d30b5c6fc14fa0ec2dc1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37b141504a40cc0be937f0d5bf071fef24759171"
        },
        "date": 1789038025783,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 385061.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 607319.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 436205.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 449794,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f860acc46471dee85057842a8611895cea64b88d",
          "message": "Merge pull request #343 from LoveDaisy/feat/gui-print-mode-label-ink\n\nfix(gui): draw overlay label text as ink under the print tone",
          "timestamp": "2026-09-10T20:57:53+08:00",
          "tree_id": "f3daf403c69c5adec882772328d1a77ec96a9215",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f860acc46471dee85057842a8611895cea64b88d"
        },
        "date": 1789045689806,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 361400.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 607430.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 499679.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 380774.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dc76b64939e2b7e7bb319dee15905da1c73ee7fa",
          "message": "Merge pull request #344 from LoveDaisy/feat/image-comparison-metric-by-layer\n\ntest: give each image comparison a ruler that matches its layer (pixel ruler, lines-only parity, block-mean PSNR)",
          "timestamp": "2026-09-11T01:37:19+08:00",
          "tree_id": "41b16f85c1532f746a24d73f00d8388af7f2060a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dc76b64939e2b7e7bb319dee15905da1c73ee7fa"
        },
        "date": 1789062625915,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 346766.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608036.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 437509.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 401895.2,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "38aff9c6a97f3fdcca9801ffeb6e1dcecf4be998",
          "message": "Merge pull request #345 from LoveDaisy/chore/release-4.5.1\n\nrelease: cut 4.5.1, and make the release a per-version backfill chore",
          "timestamp": "2026-09-11T08:06:39+08:00",
          "tree_id": "c20bb077289e78ac31076d801efb94c50cab02ad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/38aff9c6a97f3fdcca9801ffeb6e1dcecf4be998"
        },
        "date": 1789085815841,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 307557.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 606764.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 434462.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 448238.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "142e29e615d7a573006eabf610b33948e5584c98",
          "message": "Merge pull request #346 from LoveDaisy/feat/hardware-perf-distribution\n\nbuild/release: ship ISA- and GPU-matched binaries behind CPUID launchers (x86-64-v4 Linux, x86-64-v3 clang-cl Windows, sm_120 fatbin)",
          "timestamp": "2026-09-11T20:51:32+08:00",
          "tree_id": "f05bd3485353b55d626d7d9fe93091774693bb97",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/142e29e615d7a573006eabf610b33948e5584c98"
        },
        "date": 1789131609120,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 352688.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 605073.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389515.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 370417.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4fdfe61cc4b1326f60a924a669133f7c6311023d",
          "message": "Merge pull request #347 from LoveDaisy/feat/raypath-analysis-panel\n\nfeat: raypath analysis panel — dedicated non-rendering pass, ranked by chain",
          "timestamp": "2026-09-12T16:27:12+08:00",
          "tree_id": "071aa48f973504cccddec5a5f2199596e22adfc9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4fdfe61cc4b1326f60a924a669133f7c6311023d"
        },
        "date": 1789202418498,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 454449.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 597641.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 383992.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 370443.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "70eb8f5fad44336cf2b57da4d314a9aced4c8224",
          "message": "Merge pull request #349 from LoveDaisy/feat/raypath-analysis-followups\n\nRaypath analysis follow-ups: fixed-seed reproducibility, session-kind rebuild predicate, joiner glyphs, debt sweep",
          "timestamp": "2026-09-12T23:46:53+08:00",
          "tree_id": "1570515bb91fdd1a4610a3ac00dc77a6ae1f7cca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/70eb8f5fad44336cf2b57da4d314a9aced4c8224"
        },
        "date": 1789228782888,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 427219.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593937.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 637628.9,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 376566.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c106292be4122b60092a9666131dde843f639b15",
          "message": "Merge pull request #348 from LoveDaisy/feat/crystal-ray-allocation\n\nfeat(core): adaptive ray allocation across crystal entries (scene.ray_allocation)",
          "timestamp": "2026-09-13T04:19:28+08:00",
          "tree_id": "82448d3731df653e760db492e5a8794a13dc6f0c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c106292be4122b60092a9666131dde843f639b15"
        },
        "date": 1789245352047,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 438166.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 595074.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 431818.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337191.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "88e0fbf6864b1d95ba7e19c4d6660fb8c15c1f4c",
          "message": "Merge pull request #350 from LoveDaisy/chore/install-manual-refresh-and-review-minors\n\nchore: refresh the install manual, land the metric-by-layer review minors, report wrong-size anchor planes once",
          "timestamp": "2026-09-13T04:52:15+08:00",
          "tree_id": "94bf3da366cd365b6946ce53b5309cd6df95c36e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/88e0fbf6864b1d95ba7e19c4d6660fb8c15c1f4c"
        },
        "date": 1789247019730,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 460521.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592155.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 422287.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 338599.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d5c230764f43779ffb32bc75e454ec07f2159a03",
          "message": "Merge pull request #351 from LoveDaisy/fix/exposure-mode-combo-fixed-separation\n\ntest(gui): prove exposure-mode separation with an intensity probe, not a seed-dependent gap",
          "timestamp": "2026-09-13T05:13:06+08:00",
          "tree_id": "8da3f1ccd7d22ef20d6fff6dff9a5c615f8e013d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d5c230764f43779ffb32bc75e454ec07f2159a03"
        },
        "date": 1789248083352,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 474199.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592907.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 384547.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 550630,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a68880e398137ef91895edbb6d4ff0c997923e4",
          "message": "Merge pull request #352 from LoveDaisy/chore/regen-refs-deterministic-single-shot\n\nchore(regen-refs): shoot deterministic groups once, share runs across groups, refuse stale-base reshoots",
          "timestamp": "2026-09-13T05:31:09+08:00",
          "tree_id": "5e9ceaec07873c2b9154175e6d957784999b8820",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a68880e398137ef91895edbb6d4ff0c997923e4"
        },
        "date": 1789249338363,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 389497.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592514.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 425177.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 372608.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "365409776ad9761a5ebf3402cf9cde48f573e9d8",
          "message": "Merge pull request #353 from LoveDaisy/fix/render-consumer-label-flake-root-cause\n\nfix(test): root-cause the RenderConsumerLabel flake — an uninitialized SunParam azimuth",
          "timestamp": "2026-09-13T05:47:24+08:00",
          "tree_id": "2a620f7705b88d7f56d29a1cf923e190e358fd9e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/365409776ad9761a5ebf3402cf9cde48f573e9d8"
        },
        "date": 1789250195815,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 473770.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594577.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 468123.6,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 436391.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "35235a01c6962917a905abe105b458ee0ba444ab",
          "message": "Merge pull request #354 from LoveDaisy/feat/ray-num-slider-100b-log-scale\n\nfeat(gui): Rays(M) slider spans 0.1..100 000 M on a kLog track, one domain for both rows",
          "timestamp": "2026-09-13T06:17:09+08:00",
          "tree_id": "8c31257305277f134a3473abfb64eca3bdbdbc3a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35235a01c6962917a905abe105b458ee0ba444ab"
        },
        "date": 1789252142656,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 359420.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593463,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432483.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 571161.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f0add0b4a676a2e20ab27c782e9e9b5a182aac5b",
          "message": "Merge pull request #355 from LoveDaisy/feat/cli-lens-and-grid-contract\n\nfeat(lens): the CLI/GUI lens contract — short-edge fov, defaults, focal length import, annotations at intensity 0",
          "timestamp": "2026-09-13T07:31:34+08:00",
          "tree_id": "14ed237826bf2e9217fc60dc8c0bd508aee5669a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f0add0b4a676a2e20ab27c782e9e9b5a182aac5b"
        },
        "date": 1789256490985,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 404296.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594911.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 501402.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 378549.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "51fa59e29850545fd09b7f6041faaed4a4bea4cd",
          "message": "Merge pull request #356 from LoveDaisy/feat/cuda-hostgen-black-and-energy-accounting\n\nfix(cuda): host root-gen fallback renders again; landed weight reduced per warp so the energy ledger matches legacy",
          "timestamp": "2026-09-13T08:09:28+08:00",
          "tree_id": "5310d81311d369114cdde3eb97a22ec84b3b27c2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51fa59e29850545fd09b7f6041faaed4a4bea4cd"
        },
        "date": 1789258760432,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 444666.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594342.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 428027.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 443613.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "22166140295c68e58e6375028394d4587a351c11",
          "message": "Merge pull request #357 from LoveDaisy/feat/view-center-angular-dist-grid\n\nfeat(annotation): view_dist — circles of constant angular distance from the optical axis, config → core → C API → GUI",
          "timestamp": "2026-09-13T12:02:01+08:00",
          "tree_id": "6875aee1957344381ca66a902bb0dc2ba20f8f02",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/22166140295c68e58e6375028394d4587a351c11"
        },
        "date": 1789273859093,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 495013.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593779,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423887.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 576401,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "682bf9aadbd77a61c2d7697ccd4b353bb70a06be",
          "message": "Merge pull request #358 from LoveDaisy/fix/equidistant-focal-length-factor-two\n\nfix(config): equidistant lens f→fov conversion was half the documented value",
          "timestamp": "2026-09-13T12:42:20+08:00",
          "tree_id": "ad66fbc16fa5d96511d2e7667a3348c6c69b86d5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/682bf9aadbd77a61c2d7697ccd4b353bb70a06be"
        },
        "date": 1789275324609,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 337608.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 590600.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 424936.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 378988.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b10d1bbc134afb64423fa842b7740de5a59934f2",
          "message": "Merge pull request #359 from LoveDaisy/feat/analysis-panel-polish\n\nfeat(gui): raypath analysis panel polish — first-picture gate, draw layer, geometry, thousands grouping, Export CSV",
          "timestamp": "2026-09-13T14:55:21+08:00",
          "tree_id": "ffbfcccfae868fd3b9504d3d98acd998d41a1933",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b10d1bbc134afb64423fa842b7740de5a59934f2"
        },
        "date": 1789283174402,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 466913.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592792.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 494210.1,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345133.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "90d23cafd492e0f65663df5a681c439b0fa09f35",
          "message": "Merge pull request #360 from LoveDaisy/feat/analysis-standing-cpu-pool\n\nfeat(server): standing CPU analysis pool on the GPU route, woken by session kind",
          "timestamp": "2026-09-13T15:15:21+08:00",
          "tree_id": "0e81a696990d2128a156faf657717d6b2838d130",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/90d23cafd492e0f65663df5a681c439b0fa09f35"
        },
        "date": 1789284487911,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 455987,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594636.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 498534.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 379542.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0eba76808ec4e34d0e76e5c741347a0779db27c2",
          "message": "Merge pull request #361 from LoveDaisy/feat/panel-state-round-trip\n\nfeat(gui): panel-derived state round trip — analysis list freshness predicate, colour-ref layer re-indexing",
          "timestamp": "2026-09-13T15:37:56+08:00",
          "tree_id": "e7548f5a60a46f37b724f835b81f368a1b59d85a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/0eba76808ec4e34d0e76e5c741347a0779db27c2"
        },
        "date": 1789285824168,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 369745.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593579.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 426670.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 379213.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      }
    ],
    "Multi-worker Throughput": [
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1a021d0025f312751971b24b6417dc1662c14555",
          "message": "Merge pull request #263 from LoveDaisy/chore/doc-stale-state-claims\n\nMake the docs and comments say what the code actually does now",
          "timestamp": "2026-08-11T12:09:28+08:00",
          "tree_id": "1cb6721b617dd3dce5b8306763e1828b8317f689",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1a021d0025f312751971b24b6417dc1662c14555"
        },
        "date": 1786422065990,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 768126.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1164596.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 769541.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 663625.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "39bb35c6eee19cccafa61c3f38f67c65cbfec9b6",
          "message": "Merge pull request #264 from LoveDaisy/chore/test-premise-expiry-and-gate-justification\n\ntest: retire five dead observation channels and one lying marker",
          "timestamp": "2026-08-11T20:15:43+08:00",
          "tree_id": "d2f97ffb2e4eada1402851069e948df82831d91b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/39bb35c6eee19cccafa61c3f38f67c65cbfec9b6"
        },
        "date": 1786451268121,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 822113.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1167317.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 772270.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 628480.1,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "a1957e028279c05871bc50cbd1001cacb1aa2ee8",
          "message": "docs: make a completeness claim carry the same burden of proof as adding code\n\n\"Covers all 20 panels with zero omissions\" reads as an achievement and passes\nreview unchallenged; \"this class is not worth covering\" has to be argued for.\nThat asymmetry is the default state rather than anyone's choice, so completeness\nwins every conflict without a single person advocating for it -- including\nconflicts against the budget the same task committed to.\n\nPR #261 is the measured instance: a pre-committed target of -30% de-commented\ntest lines (baseline 21,336, pinned by two independent counters with 52/52 files\nzero diff) landed at -8.7%, and the coverage backfill demanded by \"20 panels,\nzero omissions\" accounts for roughly a third of the miss. Escape-defect density\nover those same files had already been measured and spans 8x; the equal-weight-\nper-panel split discarded that measurement.\n\nThe rule asks for the justification, not the reduction. Whether a leaner suite\nwould have let more defects escape is a counterfactual and untestable, so this\nis explicitly not a mandate to cut -- only a requirement that an equal-weight\npartition state its reason when a per-member value measure is available.\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>",
          "timestamp": "2026-08-12T08:19:41+08:00",
          "tree_id": "45eab6bd9bc7f45bb51575ce022fbe40ff3e9e32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a1957e028279c05871bc50cbd1001cacb1aa2ee8"
        },
        "date": 1786494692358,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 909759.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172285.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 767219.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 593607.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3b06a512d9e1c500bf53c2ae03ebc98e78a8ee6d",
          "message": "Merge pull request #267 from LoveDaisy/chore/perf-doc-machine-provenance\n\ndocs: 远程验证文档按「角色 / 主机绑定」分层，并写入新参照机 recipe",
          "timestamp": "2026-08-12T15:34:26+08:00",
          "tree_id": "622acaacaf2c20558ea2765a841267f06017335f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3b06a512d9e1c500bf53c2ae03ebc98e78a8ee6d"
        },
        "date": 1786520735029,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 896438.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1179681.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 724165.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 679810.9,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "660cb36f3211d40db009ebe4cb974d819900051d",
          "message": "Merge pull request #268 from LoveDaisy/task/msvc-portability-test-env-helper\n\nfix(test): 把 setenv/unsetenv 的 #ifdef 收敛成一个有名字的 helper",
          "timestamp": "2026-08-12T20:45:55+08:00",
          "tree_id": "a60dffae6ac4ce8bfff90f14b0a1c3d1815c8fe7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/660cb36f3211d40db009ebe4cb974d819900051d"
        },
        "date": 1786539477912,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 1180440.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 759994.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 649292.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8684aa9e5d304592bc3fe6a782307b7a9da8c704",
          "message": "Merge pull request #269 from LoveDaisy/task/win-static-crt-cmp0091\n\nfix(build): 让 CMP0091 真正生效，Windows 发布产物链接静态 CRT",
          "timestamp": "2026-08-13T17:00:34+08:00",
          "tree_id": "9e6e68dabd87f16fcb817b3f0ef3dcfef9246ada",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8684aa9e5d304592bc3fe6a782307b7a9da8c704"
        },
        "date": 1786612350123,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 758248.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1174354.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 870002.9,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 599844.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7da5553043fc957ef127227fd09c6bd2a434969a",
          "message": "Merge pull request #270 from LoveDaisy/docs/gui-blueprints\n\ndocs(gui): 落盘视觉语言与布局架构两份 GUI 蓝图",
          "timestamp": "2026-08-14T01:14:13+08:00",
          "tree_id": "d80b51f210e34913cdcdae277891c8370b111d51",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7da5553043fc957ef127227fd09c6bd2a434969a"
        },
        "date": 1786641955578,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 841518.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175661.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 764256.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 646875.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "de2400d72e0c74815654b5c0798537d321c36342",
          "message": "Merge pull request #271 from LoveDaisy/task/gui-visual-language\n\nfeat(gui): 落地 GUI 视觉语言——单一 owner、比例字体、量化节奏、调色板与语义色",
          "timestamp": "2026-08-14T08:18:19+08:00",
          "tree_id": "e4dae219cff3b357fa69562f4581b9a13c1244c7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de2400d72e0c74815654b5c0798537d321c36342"
        },
        "date": 1786667412090,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 835960.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1167263.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 725564.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 651206.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e5a855759f2a1e535cf1ff56144d57a93472b4c1",
          "message": "Merge pull request #272 from LoveDaisy/feat/new-gui-layout\n\nfeat(gui): 新 GUI 布局——「文档 | 图像 | 运行」三区重组（集成分支）",
          "timestamp": "2026-08-18T13:27:49+08:00",
          "tree_id": "4d399707fc655846f8bae9083488c3c91c9ce3aa",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e5a855759f2a1e535cf1ff56144d57a93472b4c1"
        },
        "date": 1787031516782,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 756212.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1178269.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 905969.2,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 596652.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a66c523b420650ecb6f8abbe787e38f506ba4e3",
          "message": "Merge pull request #273 from LoveDaisy/feat/gui-form-refinement\n\nfeat(gui): 控件形态精修——宽度 token、PropertyRow 与排版秩序",
          "timestamp": "2026-08-19T02:57:24+08:00",
          "tree_id": "732853fea0235432d76a2465b680be58ccc15e51",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a66c523b420650ecb6f8abbe787e38f506ba4e3"
        },
        "date": 1787080142340,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 877737.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1155777.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1267538.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 601092.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "f1228505c4ed659c006158629ed8b4501eba7074",
          "message": "docs(gui-layout): 记录内测否决，蓝图从待办降为设计记录\n\nv4.4.2（老 shell）与 v4.4.2-new（新 shell）小范围内测对比后，几乎全部\n内测用户选择回到老 shell。main 回退到 PR #271：视觉语言层留下，形态层\n（PR #272 shell 重组 + PR #273 控件精修）退出，实现保存在分支\nfeat/new-gui-layout 与 tag/release v4.4.2-new。\n\n三处改动都是为了让下一个读者不把已被否决的方向当成在途的待办：\n\n1. gui-layout-architecture.md 顶部状态改写 + 新增 §8。记下三件事：反馈\n   粒度未知（聚合结论没区分拒的是形态还是外观，故保留视觉语言层既不由\n   它支持也不被它否定，下一步取证是老 shell 上单发视觉语言层做窄 A/B）；\n   方法层教训（原型验收与 owner 上手两道闸共享同一盲区——都在问「形态\n   本身好不好」，没问「熟练用户是否愿意换」，而后者才是内测在问的）；\n   以及没有被否决的部分（§0 诊断对今天的老 shell 仍为真，§5 六条被推翻\n   形态不恢复候选资格）。\n\n2. gui-visual-language.md 更正时态。该文 §4 定案随 PR #271 留在 main 上，\n   但文中多处把 docking 迁移写成在途的事，回退后不再成立，一律改读作\n   「将来任何一次面板重排」。同时补回 §7 正文字体的收口——字体定案\n   （Roboto Medium 15 构建期嵌入）随 PR #271 落地，而写下这条收口的文档\n   改动落在 PR #273 里，被本次回退一并带走，留下文档说「未定案」而代码\n   已定案的漂移。\n\n3. AGENTS.md 两条索引同步。索引是这两份文档唯一的必经检索入口，否决\n   记录只写在文档里而不写在索引上，等于没写。",
          "timestamp": "2026-08-26T10:15:35+08:00",
          "tree_id": "0508524749ee48fed4c9dbf374750ff51a0579f3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1228505c4ed659c006158629ed8b4501eba7074"
        },
        "date": 1787711362646,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 993852.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1176109.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 721449.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "840bc16a46aa7fc6e6265bd34609af330a10af64",
          "message": "docs(gui-layout): 形态层的锚点从 v4.4.2-new 标签改为分支 + commit\n\n内测反馈已到手，v4.4.2-new 标签与 release 随之删除（留着它就是把已被\n否决的界面挂在 Latest release 上发给外部用户）。但 §8 与 AGENTS.md 索引\n都拿这个标签当「形态层保存在哪里」的锚点，标签一删锚点就悬空。\n\n改为锚在分支 feat/new-gui-layout 与 commit 7a66c523——commit hash 是\n永久锚点，分支是可读入口。§8 同时留一句说明标签删除的原因和重出该构建\n的办法（从该分支重新打标签），免得下一个读者以为构建丢了。",
          "timestamp": "2026-08-26T10:28:26+08:00",
          "tree_id": "c793fa450b5d26416c5676defa6dd7ccb9d2badc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/840bc16a46aa7fc6e6265bd34609af330a10af64"
        },
        "date": 1787712082130,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 851851.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175742.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 761624.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 643315.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "9be4b99e4907e79992121a4e19f6dd742ba03dea",
          "message": "docs(gui-layout): 原型取证锚点随原型分支一同退役\n\n三个 spike 分支（gui-layout-prototype / gui-visual-language / imgui-docking）\n从未推送、只存在于本地，随新布局方向被内测否决一并删除。布局蓝图开头\n把其中 gui-layout-prototype 及三个 commit 写作「取证锚点」，分支一删这行\n就指不到任何东西。\n\n改为如实说明：取证现场已不存在，§1–§5 此后是已记录的判断而非可重新核验\n的断言；要看那一版形态实际长什么样，去 feat/new-gui-layout——同一形态的\n完整实现，完成度高于原型，只不含 §5 那六条从未被实现的候选。\n\n§5 开头补一句界定：六条结论不因原型删除而撤销，重提的一方承担举证责任。\n顺带修一处漂移：视觉语言 §4.1 仍写着「具体字体尚未定案」，而 §7 的收口\n和 main 上的代码都已是 Roboto Medium 15。",
          "timestamp": "2026-08-26T11:34:40+08:00",
          "tree_id": "f4517359a5286de30e0bd6f6f01ac44f5b0b416e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9be4b99e4907e79992121a4e19f6dd742ba03dea"
        },
        "date": 1787715959733,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 815579,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1163503.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 919002.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 583903.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "41354876b363944c8882dba3c46014eb797382aa",
          "message": "Merge pull request #274 from LoveDaisy/task/bg-image-filtering\n\nfix(gui): 底图纹理改用 mipmap + trilinear，修缩小显示时的欠采样混叠",
          "timestamp": "2026-08-26T20:00:46+08:00",
          "tree_id": "a5fbb679ee0ffcf9c28c9aec23ea64e6663e04a8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/41354876b363944c8882dba3c46014eb797382aa"
        },
        "date": 1787746339827,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1067887.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1156849.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 721951.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 653202,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f17b841b725042bd28408b6db3fd1da5cfc7fa8f",
          "message": "Merge pull request #275 from LoveDaisy/task/bg-image-transform\n\nfeat(gui): 底图可平移缩放，让裁剪过的照片能与仿真结果对齐",
          "timestamp": "2026-08-26T20:20:53+08:00",
          "tree_id": "24754ad01298b2e39006a4bc1d728369562e429c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f17b841b725042bd28408b6db3fd1da5cfc7fa8f"
        },
        "date": 1787747561436,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 807604.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1170701.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 926797.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 655167.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e23d669872bbfb1762a95659960d412d793c55e9",
          "message": "Merge pull request #276 from LoveDaisy/task/crystal-enable-toggle\n\nfeat(gui): 晶体卡新增「参与仿真」toggle，替代把权重拖到 0",
          "timestamp": "2026-08-26T20:39:00+08:00",
          "tree_id": "4b0f2abbc5fcc90f68d6ae6281bfef446c82cc03",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e23d669872bbfb1762a95659960d412d793c55e9"
        },
        "date": 1787748631266,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 997764.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1168653.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 767739.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 654684.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "423ff22e33a81fc6521fb8da4f55037e88e7ac93",
          "message": "docs(gui-layout): 内测反馈的细粒度到手，改掉三处已被它推翻的记录\n\n上一轮记录写于反馈只有聚合结论时，有三处现在是错的，且都写在下一个读者\n必经的位置上。\n\n1. 「反馈粒度未知」作废。细粒度反馈是：配色被接受（用户对配色的接受范围\n   很宽），被拒的是形态，理由具体——不如老 plain 布局一眼看到所有信息，\n   典型操作「同时快速调整冰晶与太阳高度」要多点好几步、来回切换不便。\n   于是原计划的「老 shell 上单发视觉语言层做窄 A/B」不必做了：那道取证是\n   为了问出粒度，粒度已经有了。\n\n   机制不是打磨不足，是 master-detail 的结构性代价：老 shell 左栏晶体卡与\n   右栏 Scene（含太阳）永久同时在屏，导航成本为零；新形态里太阳是检视器的\n   一个 page，晶体是同一检视器的另一个 page，一次只看得见一个对象。\n\n2. 方法层教训改写。原先写作「两道闸共享盲区＝没问熟练用户是否愿意换，\n   验收链必须含一条能测迁移成本的证据」——这个说法经不起推敲：本地开发阶段\n   必然只能问「这个形态好不好」，必然要发版才拿得到用户反馈，那不是一道\n   本可设而没设的闸，而是结构性事实；发版、拿反馈、便宜回退、分支留存，\n   这个环当时是通的。真正的偏差是验收问错了量——两道闸问的都是「形态本身\n   好不好用」，而用户答的是两个本地就能机械量出来却从没被量过的数：常见\n   任务的操作步数，以及一屏同时可见的字段集合。\n\n3. §0 第一条补一个维度。§0 骂老 shell「分割轴任意」属实，但「任意」不等于\n   「差」：那条任意的轴恰好让最常一起调的两组永久同屏，而 §0 从未度量过\n   同屏可见性这一维，用户却只在这一维上表了态。因此追加一条硬约束——将来\n   任何一次重排，最常一起调的字段组必须保持同屏可见——地位等同 §5 那六条\n   被推翻形态。\n\ngui-visual-language.md 顶部同步：那条「不构成外观已被接受的证据」作废，但\n边界要写清楚，被问到的只有配色，§4 其余条目仍只是「没有被反对」。\nAGENTS.md 两条索引一并同步——索引是这两份文档唯一的必经检索入口。\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>",
          "timestamp": "2026-08-26T20:49:35+08:00",
          "tree_id": "6b66463dd3987722c0987a1f07a416d5332cca1b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/423ff22e33a81fc6521fb8da4f55037e88e7ac93"
        },
        "date": 1787749583230,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 859159.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1183857,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 720033.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 711061.4,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "eea0a7b0268762f9590f8d50912a3662a80e6936",
          "message": "Merge pull request #277 from LoveDaisy/task/gui-overlay-table\n\nfeat(gui): Overlay 辅助线组改为 6 列表格形态",
          "timestamp": "2026-08-29T12:19:58+08:00",
          "tree_id": "7e912e59f4173ec789186af3f24b4e7329ddef8d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/eea0a7b0268762f9590f8d50912a3662a80e6936"
        },
        "date": 1787977871287,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1177114.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1183727.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 818111.4,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 604573.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ff02ed3a487ac6e46dd2720351057ed2314aa60b",
          "message": "Merge pull request #278 from LoveDaisy/task/full-sphere-roll-flip\n\nfix(core+gui): 全球面快路径补 roll 旋转对称条件，修滑条端点浮点漂移导致的采样路静默切换",
          "timestamp": "2026-08-29T13:20:57+08:00",
          "tree_id": "12a2380afc7b07d8b15f664fa3f2bf288d3e7f1e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ff02ed3a487ac6e46dd2720351057ed2314aa60b"
        },
        "date": 1787981569517,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 932398.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172257.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 934856.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 652079.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6320bbab9d388df3b07a528374e61df82d920a6b",
          "message": "Merge pull request #279 from LoveDaisy/task/gui-theme-color-closure\n\nrefactor(gui): 颜色收口——色槽补齐 58/58 + 调用点裸字面量逐处 disposition",
          "timestamp": "2026-08-29T15:06:01+08:00",
          "tree_id": "082d1f2e575f42837f85a5f180ed8d57c1ca7176",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6320bbab9d388df3b07a528374e61df82d920a6b"
        },
        "date": 1787987825114,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 827273.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1163973,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 719872.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 604701.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "994823ec3a42f7c5fb27150249b58b30ac4f6336",
          "message": "Merge pull request #280 from LoveDaisy/task/overlay-table-acceptance-fixes\n\nfix(gui): Overlay 表格人工验收三条修复",
          "timestamp": "2026-08-29T22:17:55+08:00",
          "tree_id": "6d0628441baf2aabfb41489d69d226d3905d11b0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/994823ec3a42f7c5fb27150249b58b30ac4f6336"
        },
        "date": 1788013779495,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 957552.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1165039.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 946711.5,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 648029.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "26e730907f491d40869033a4ce67ac66edc88f8a",
          "message": "Merge pull request #281 from LoveDaisy/task/gui-label-column-gap-alignment\n\nfix(gui): 行末标签列左缘对齐 + 间距收敛为单一 owner",
          "timestamp": "2026-08-30T11:28:21+08:00",
          "tree_id": "51266763b2f5e9a7423d99f0a75aaf7d05257f11",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/26e730907f491d40869033a4ce67ac66edc88f8a"
        },
        "date": 1788061024255,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1183458.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1159698.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 952919,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 778281.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "37658798751c385f405d25527fc3226fb807ff08",
          "message": "Merge pull request #282 from LoveDaisy/task/gui-entry-card-layout-and-crystal-identity\n\n晶体卡片 layout 重排 + 晶体身份可寻址 + Colors 面板编号/失效态",
          "timestamp": "2026-08-30T13:34:44+08:00",
          "tree_id": "23118e5367b29ce0c82bda8278360f565c79c8b8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37658798751c385f405d25527fc3226fb807ff08"
        },
        "date": 1788068775565,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1123679.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1156708.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 770706.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 658867.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "d63befda26e1912dcf6d6d8cc67efd7f34eb7ac2",
          "message": "bump patch version for release",
          "timestamp": "2026-08-30T17:17:40+08:00",
          "tree_id": "ae2567feaaad662c625ed888e28482837014d4c8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d63befda26e1912dcf6d6d8cc67efd7f34eb7ac2"
        },
        "date": 1788082221153,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 763856.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1162533.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 936707.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 784105.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "53299d835046ba74d6f2897c4a1566993368a6bf",
          "message": "Merge pull request #283 from LoveDaisy/task/gui-fisheye-lens-border\n\nGUI: 鱼眼镜头有效区边框辅助线",
          "timestamp": "2026-08-30T22:06:58+08:00",
          "tree_id": "2ba992405cad4908ee5cc797c9404a38a110ff48",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/53299d835046ba74d6f2897c4a1566993368a6bf"
        },
        "date": 1788099496447,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 846693.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172929.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 760501,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 629884.1,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "68f3be6a4fc6353c202b92f6538e7b9c3d5d1100",
          "message": "Merge pull request #284 from LoveDaisy/task/retire-comma-raypath-separator\n\n退役 raypath 逗号连接符：静默算错改为指名改法的拒绝 + 加载期迁移",
          "timestamp": "2026-08-30T22:29:34+08:00",
          "tree_id": "e9cc435a031fe70e00ae4ab2e1c43c075f39eb24",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/68f3be6a4fc6353c202b92f6538e7b9c3d5d1100"
        },
        "date": 1788100791417,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1187522.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1173980.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 726593.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 674099.1,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c798112109aa5fb9e2f85564fd0f9bfb59e0259",
          "message": "Merge pull request #285 from LoveDaisy/task/user-defaults-schema-version\n\n给 user_defaults.json 盖上独立的 schema 版本戳（只记录，不设闸，不迁移）",
          "timestamp": "2026-08-31T08:46:01+08:00",
          "tree_id": "c6063d0d04375864505dbcfd05c8f5eba511d092",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6c798112109aa5fb9e2f85564fd0f9bfb59e0259"
        },
        "date": 1788137858703,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 904543.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1177620.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 772439.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 657129.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "034ab22193bfe8cf4e9efb4aac1599e82c5e3308",
          "message": "Merge pull request #286 from LoveDaisy/feat/adjustable-background-color\n\n可调背景颜色：GUI/CLI 五路一致 + core 定义域掩码 + 注记层处置",
          "timestamp": "2026-08-31T12:48:11+08:00",
          "tree_id": "465ab3c2e3bc53104f4df724fba2c95cd70a2d2c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/034ab22193bfe8cf4e9efb4aac1599e82c5e3308"
        },
        "date": 1788152408554,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 950297.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1154582,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 808975.9,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 652505.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8054154bedaaf1ce926cd9c7d44a6c1c548cc115",
          "message": "Merge pull request #287 from LoveDaisy/feat/absolute-ev\n\nfeat: 绝对 EV —— cross-simulation 可比的曝光尺度",
          "timestamp": "2026-08-31T15:03:03+08:00",
          "tree_id": "e66dfd46e67488974ac2ab7f738dc49b2e56328a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8054154bedaaf1ce926cd9c7d44a6c1c548cc115"
        },
        "date": 1788160553120,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 792118.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1168112.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1143818.6,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 646922.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5360f028303b6963e69eb26ba999c203a3f3018e",
          "message": "Merge pull request #288 from LoveDaisy/feat/cli-gui-render-parity\n\n让导出的 config 诚实描述用户所见 + 建 CLI↔GUI 出图对照闸",
          "timestamp": "2026-09-01T10:20:29+08:00",
          "tree_id": "1d4a30db33e4386ab9a58ed55adc7521892bc4bb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5360f028303b6963e69eb26ba999c203a3f3018e"
        },
        "date": 1788230000112,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 807868.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1159192.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 765574.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 647953.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "93c163fe3b8be8017bff712da303b6afa9ba8c03",
          "message": "Merge pull request #289 from LoveDaisy/task/lens-json-names-oob\n\nfix(gui): 修 kLensTypeJsonNames 越界读（用户可达崩溃）",
          "timestamp": "2026-09-01T11:38:10+08:00",
          "tree_id": "386dbbee06c0b18a0469f3dd58b480cc1f8c7e05",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/93c163fe3b8be8017bff712da303b6afa9ba8c03"
        },
        "date": 1788234597490,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1069350,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1162386.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 766857.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 603606.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9faf145b0edde6af078573ca8726d7ce246411df",
          "message": "Merge pull request #290 from LoveDaisy/task/preview-solid-angle-jacobian\n\nfeat(gui): 预览 shader 补上目标镜头的相对照度，使非等面积投影下 GUI 与 CLI 可逐像素比",
          "timestamp": "2026-09-01T13:00:21+08:00",
          "tree_id": "89c8de38143255c6034f17752c7461109dd74f23",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9faf145b0edde6af078573ca8726d7ce246411df"
        },
        "date": 1788239567153,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 948051,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1148494,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 935366,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 651091.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "03e21b4dd2baadc1325958ed61d1293e3d6434c9",
          "message": "Merge pull request #291 from LoveDaisy/feat/fisheye-domain-widening\n\nfeat(core): 单镜头鱼眼定义域按 lens 放宽到 θ≤180，与 GUI 对齐",
          "timestamp": "2026-09-01T15:17:01+08:00",
          "tree_id": "db0ee29334ec16918d95c3230818b73c6aa64f80",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/03e21b4dd2baadc1325958ed61d1293e3d6434c9"
        },
        "date": 1788247628285,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1049482.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1166530.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1000231.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 795330.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a6034051812988c3e1fb7639296b5d3fbdbf8375",
          "message": "Merge pull request #292 from LoveDaisy/feat/core-annotation-layer\n\nfeat(core): 注解层补齐——辅助线与文字 label 收敛为 core 单一来源",
          "timestamp": "2026-09-02T04:45:19+08:00",
          "tree_id": "fe59f2f65430792483491f011bdddca70953ff32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a6034051812988c3e1fb7639296b5d3fbdbf8375"
        },
        "date": 1788296271431,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 994729.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1166710.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 759458.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 610265.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fc5de377009120dd26063703a16f31027aaffd62",
          "message": "Merge pull request #293 from LoveDaisy/feat/test-time-and-scope-discipline\n\nfeat(ci/doc): 测试时间预算的 owner —— 实测拓扑、分片重装箱、分层契约",
          "timestamp": "2026-09-02T09:04:32+08:00",
          "tree_id": "cfe6beb5c25bbb4014f67bf142635e635a347f4e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc5de377009120dd26063703a16f31027aaffd62"
        },
        "date": 1788311805928,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1019010.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1161234.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 763810.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 641346.7,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "32515f9970bfd614540c36dece2c55fe71eee1a6",
          "message": "Merge pull request #294 from LoveDaisy/chore/annotation-doc-and-diagnostics-gaps\n\nchore(doc/cli): 补 zenith_nadir schema 文档；renderer 超限诊断指向真正的上限",
          "timestamp": "2026-09-02T13:14:20+08:00",
          "tree_id": "377cc3a2fbdf13228f8c678cf3391e0e9631022c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/32515f9970bfd614540c36dece2c55fe71eee1a6"
        },
        "date": 1788326822780,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1128220.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1161606,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 755402.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 653198.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "18624004d00f891498779bf12248f36046859a41",
          "message": "Merge pull request #295 from LoveDaisy/feat/fast-e2e-dominant-test\n\ntest(e2e): smoke 按 config 拆成独立 pytest item —— 收集粒度对齐调度粒度，零覆盖损失",
          "timestamp": "2026-09-02T14:42:18+08:00",
          "tree_id": "c633f9361f8df5a4557dd160ab557794d6e71044",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/18624004d00f891498779bf12248f36046859a41"
        },
        "date": 1788332015631,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 812864.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1161578.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 844346.3,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1ac3ab63b1d72eb8034782f0b8b8f09e100b4636",
          "message": "Merge pull request #296 from LoveDaisy/task/save-open-visual-consistency-red\n\nfix(gui): .lmc 与 composite 纹理改存纯辐亮度，渐晕由显示端统一补上",
          "timestamp": "2026-09-02T18:00:52+08:00",
          "tree_id": "b7bbed427787b088d61b2fa2e14ac122e1150a1d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1ac3ab63b1d72eb8034782f0b8b8f09e100b4636"
        },
        "date": 1788344054709,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 880012.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1163211.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 817114.3,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 645993.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "08aa283330101f2ed499cab2ab657abe4bcbb2b2",
          "message": "Merge pull request #297 from LoveDaisy/feat/lens-projection-semantics\n\nfeat(core): 收口 ProjectExitToPixel 遗留的三条 core↔GUI 分歧（参考图只重拍一次）",
          "timestamp": "2026-09-02T23:11:02+08:00",
          "tree_id": "db3c8f71967ab4435d5b915e7c9047561643c424",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/08aa283330101f2ed499cab2ab657abe4bcbb2b2"
        },
        "date": 1788362667544,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1140306.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1145637.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1297627.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 643500.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dfe34bc13b012f367d913460738f8dc02a50faa0",
          "message": "Merge pull request #298 from LoveDaisy/chore/gui-unit-heartbeat-wallclock-margin\n\ntest(gui-unit): 心跳用例改 wait-until，墙钟余量 250ms → 秒级",
          "timestamp": "2026-09-03T01:12:40+08:00",
          "tree_id": "1b9799920e428da55ee87172d76c0d6689c879f4",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfe34bc13b012f367d913460738f8dc02a50faa0"
        },
        "date": 1788369962796,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1063222.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1168980.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 936495.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 650885,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "82310d302119a07fff51338e811ffd189ee0aac1",
          "message": "Merge pull request #299 from LoveDaisy/feat/relative-ev-anchor\n\nfeat(core): 把 relative 曝光锚点做对 —— 锚到固定全天缓冲，CLI 与 GUI 消费同一个数",
          "timestamp": "2026-09-03T15:07:51+08:00",
          "tree_id": "582e122487ca1d78c26a6dae0bb7dc213f067af6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/82310d302119a07fff51338e811ffd189ee0aac1"
        },
        "date": 1788420079015,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 793100.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1166566.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 914914.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 644462.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c3c5042e4f17c60ecba3cbf6ff38267594826fa",
          "message": "Merge pull request #300 from LoveDaisy/fix/scene-cnt-publish-ordering\n\nfix(server): 记账先于发布，消除批次静默丢失的竞态窗口",
          "timestamp": "2026-09-03T19:19:00+08:00",
          "tree_id": "a4a02453a42967340bf873f3b689410b89d032b9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6c3c5042e4f17c60ecba3cbf6ff38267594826fa"
        },
        "date": 1788435088458,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 872614.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1168252.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 743617.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 609238.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "365ba58f71d4354ec1035328c12ffd3427c6492e",
          "message": "Merge pull request #301 from LoveDaisy/feat/crystal-slider-domain-and-format\n\nfeat(gui): 柱晶高度域扩到 1e-4，滑杆显示格式由映射闭式定下界并在编译期判错",
          "timestamp": "2026-09-04T15:31:05+08:00",
          "tree_id": "c649e8f7c8eff961a4f64eb3b5956f027157ad32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/365ba58f71d4354ec1035328c12ffd3427c6492e"
        },
        "date": 1788507859297,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 984330.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1161256.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 720322.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 638619.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ddc94c02faa23de01ef59b90d25f106843699744",
          "message": "Merge pull request #302 from LoveDaisy/fix/preview-source-gather-half-texel\n\nfix(gui): 预览 shader 从 dual-EA 源纹理取样时多出的半个纹素",
          "timestamp": "2026-09-04T18:44:53+08:00",
          "tree_id": "5bbccc2a028e253d4dade882c65deb804b594261",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ddc94c02faa23de01ef59b90d25f106843699744"
        },
        "date": 1788519443814,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 960502.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1161010.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 747224.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 600220.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7381ca162338ea5105412cc97719e7c7d7eeb1cb",
          "message": "Merge pull request #303 from LoveDaisy/feat/alloc-churn-and-rng-bounds\n\nperf(core): all_data 缓冲区复用 + GetUniform 下标边界收敛到单一 owner",
          "timestamp": "2026-09-04T20:16:18+08:00",
          "tree_id": "f4f4cdc4118328fb3d085e6dd57776fb9c3f53b7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7381ca162338ea5105412cc97719e7c7d7eeb1cb"
        },
        "date": 1788524964883,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 917659.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1212881.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 822526.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 686498.9,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1271f2699d6339b658ee96d3cf0502055c2b034a",
          "message": "Merge pull request #304 from LoveDaisy/fix/gui-preview-single-render-path\n\nfix(gui): 预览与 Screenshot 导出收敛到单一离屏 FBO 渲染路径",
          "timestamp": "2026-09-04T20:53:45+08:00",
          "tree_id": "1ea3fd858b1979513692e7dd728217e33f1635d7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1271f2699d6339b658ee96d3cf0502055c2b034a"
        },
        "date": 1788527141730,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 947317.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1211327.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 822097.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61328fbedb7c6acfaf163c040563f24285d52ff2",
          "message": "Merge pull request #305 from LoveDaisy/feat/sky-reference-points\n\nfeat(gui,core): 天空参考点标记六点泛化 + Look At 视角预设，共享 core 单源方向表",
          "timestamp": "2026-09-05T02:15:06+08:00",
          "tree_id": "66f5811bd94711b21e96cefdfa28e8d5b7dcf841",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61328fbedb7c6acfaf163c040563f24285d52ff2"
        },
        "date": 1788546558623,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 929675.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1211266.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 761270.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 672831.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9327bb7a34e0095c493f6408acc9067e2512fd61",
          "message": "Merge pull request #307 from LoveDaisy/fix/gui-test-harness-gates\n\ntest(gui): 闭合 gui_test 现场三处守卫可信度缺口——恒红闸 / 缺失的互比闸 / 注释纪律一致性",
          "timestamp": "2026-09-05T14:10:18+08:00",
          "tree_id": "2ca052b6512de4cebc819bec0838eb5723d8a739",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9327bb7a34e0095c493f6408acc9067e2512fd61"
        },
        "date": 1788589263729,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 806841.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1213671.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 817336,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "1f3a9c864ad290679f4f638c62e9bbf6be03a855",
          "message": "Merge pull request #307 from LoveDaisy/fix/gui-test-harness-gates\n\ntest(gui): 闭合 gui_test 现场三处守卫可信度缺口——恒红闸 / 缺失的互比闸 / 注释纪律一致性",
          "timestamp": "2026-09-05T14:15:53+08:00",
          "tree_id": "2ca052b6512de4cebc819bec0838eb5723d8a739",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1f3a9c864ad290679f4f638c62e9bbf6be03a855"
        },
        "date": 1788590062750,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 922332.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1215765.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 827784.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 635446.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fa044a6142525e887ef3e31b89027a953f00343b",
          "message": "Merge pull request #308 from LoveDaisy/fix/clamp-uniform-zero-n\n\nfix(core): ClampUniformToIndex 的 n==0 前提由 assert 改为 FatalAbort 硬守卫",
          "timestamp": "2026-09-05T14:40:02+08:00",
          "tree_id": "b9b2c7baef1b585fa751319ac718c1a01132d770",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa044a6142525e887ef3e31b89027a953f00343b"
        },
        "date": 1788591151043,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 890579.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1215507,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 822392.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 638937.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "92c4036ebd9dc10187397db16e7a435ba69af19b",
          "message": "Merge pull request #309 from LoveDaisy/chore/native-arch-measurement-hygiene\n\nfeat(bench): 让「本地构建不是出货二进制」这件事在取数字的地方可见（ISA 出处）",
          "timestamp": "2026-09-05T14:54:12+08:00",
          "tree_id": "d6e704e339bd4d14891a0549aafe20bd3799c40e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/92c4036ebd9dc10187397db16e7a435ba69af19b"
        },
        "date": 1788592057019,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1100868.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217079.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 820210,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 676469.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5aea061e953c620bfe6709bf6f2ce87103402f05",
          "message": "Merge pull request #310 from LoveDaisy/fix/cli-label-viewport-clamp\n\nfix(cli): overlay label 视口 clamp 收敛为 GUI/CLI 共享的单一实现",
          "timestamp": "2026-09-05T15:09:23+08:00",
          "tree_id": "f073e0e74a8ffe5cbb3e2967c3330a884cb4279c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5aea061e953c620bfe6709bf6f2ce87103402f05"
        },
        "date": 1788592959418,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1154586.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1213367.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 823912.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 664572,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f1822e52c6d59ab2c203aa7ef388cb2b50f4e2df",
          "message": "Merge pull request #311 from LoveDaisy/feat/gpu-backend-user-preference\n\nfeat(gui): \"Use GPU\" 可存为个人默认（覆盖文件新增 app 根键），工厂默认仍是 CPU",
          "timestamp": "2026-09-05T15:44:47+08:00",
          "tree_id": "b61f1340307f90b27cb42c9f0d3c8a930fc8a63c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1822e52c6d59ab2c203aa7ef388cb2b50f4e2df"
        },
        "date": 1788594958617,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 864156,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1212358.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1361934.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 759321.4,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5015321af574f4bc5efa1787fa3feefc87a835b",
          "message": "Merge pull request #312 from LoveDaisy/fix/benchmark-steady-window-outlier\n\nfix(bench): active_short 不得拿 IDLE 检测延迟当分母（14-29x 野值的根因）",
          "timestamp": "2026-09-05T15:59:52+08:00",
          "tree_id": "e502d8ac59d36d9acdffade736d7bf32b619fa08",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5015321af574f4bc5efa1787fa3feefc87a835b"
        },
        "date": 1788595945017,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1241991.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1212952.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 821812.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fdf903868b533c085c845322ee50722158e3a089",
          "message": "Merge pull request #313 from LoveDaisy/scrum/sentinel-liveness-audit\n\ntest(sentinel): retire the one regression sentinel that outlived its mechanism",
          "timestamp": "2026-09-06T00:54:43+08:00",
          "tree_id": "58522174f803a4d43425b4cd8e96c32dbab08dee",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fdf903868b533c085c845322ee50722158e3a089"
        },
        "date": 1788628074947,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 1205476.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 757574.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 674237.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "aa3ec3edc826ec5ba4b7cb59fb1e102222309889",
          "message": "Merge pull request #314 from LoveDaisy/scrum/worker-and-batch-granularity\n\nperf(cpu): cap the automatic worker count at 10, expose --workers, and fix a hit-loop buffer overflow",
          "timestamp": "2026-09-06T10:48:35+08:00",
          "tree_id": "0f52437edfb35a55cb432bf05c2ba64797af8a5f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/aa3ec3edc826ec5ba4b7cb59fb1e102222309889"
        },
        "date": 1788663722670,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1259205.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1221126.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1018339.5,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 673528.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5cb81a9565ff0e1394a957d97ad2078d0d8f9310",
          "message": "Merge pull request #315 from LoveDaisy/scrum/changelog-backfill-and-release-notes\n\ndocs(release): 回填 v4.1.4 起 31 个版本的 CHANGELOG，并把它接进发版链路",
          "timestamp": "2026-09-06T16:24:48+08:00",
          "tree_id": "079d5a016ac9ac51339233ba0779369c19e64745",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5cb81a9565ff0e1394a957d97ad2078d0d8f9310"
        },
        "date": 1788683897234,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 837040.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217120.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 854169.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 686534.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c04ad137d42e4a3ed51cd137b75d2d97092b7864",
          "message": "Merge pull request #316 from LoveDaisy/fix/msvc-string-literal-limit\n\nfix(gui,ci): 拆开超 MSVC 上限的 shader 字面量 + 立静态门禁 + CI 触发去重",
          "timestamp": "2026-09-06T23:22:44+08:00",
          "tree_id": "6cd1e5d94251d724205eb0763696031080e86c9d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c04ad137d42e4a3ed51cd137b75d2d97092b7864"
        },
        "date": 1788708808149,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 835865.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1218279.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 829130,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 989983.4,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "349656ac94b6b2fd27ecbf4ed35812bea7646a2a",
          "message": "Merge pull request #317 from LoveDaisy/ci/windows-release-image-unify\n\nci: build Windows on the image we actually release from",
          "timestamp": "2026-09-07T01:04:20+08:00",
          "tree_id": "d7e945127aac502e4a16adeca4de66c2fca33ff5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/349656ac94b6b2fd27ecbf4ed35812bea7646a2a"
        },
        "date": 1788714901687,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1146045.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217253.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 829440.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 629547.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "59208e341a7a63e1f22366ec0fbc47211fd93950",
          "message": "Merge pull request #318 from LoveDaisy/test/e2e-cost-and-oracle-audit\n\ntest(e2e): 按「每个测试为自己的开销举证」审计套件成本，恢复预算余量",
          "timestamp": "2026-09-07T04:18:31+08:00",
          "tree_id": "a180e65114ec4dbebe9febd562ebcb9d7dcb6dcd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/59208e341a7a63e1f22366ec0fbc47211fd93950"
        },
        "date": 1788726578238,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1096881.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1214833.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 902823.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 673985,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e707b15d31a6676c8f4147a0b0cfe62dfc452995",
          "message": "Merge pull request #319 from LoveDaisy/fix/gui-entry-delete-vs-open-editor\n\nfix(gui): keep the edit modal bound to its entry across a delete",
          "timestamp": "2026-09-08T11:14:03+08:00",
          "tree_id": "493abe69703c95a59432e4a6f6623947ade2f407",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e707b15d31a6676c8f4147a0b0cfe62dfc452995"
        },
        "date": 1788837910236,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1168034,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1219054.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 762441.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 627060.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "da9e5533acc8c01c61877b6613ddf40bdce9a8b4",
          "message": "Merge pull request #320 from LoveDaisy/fix/cuda-zero-ray-batch-poisons-backend\n\nfix(cuda): stop a zero-ray layer from poisoning the CUDA backend",
          "timestamp": "2026-09-08T17:13:37+08:00",
          "tree_id": "915905aba2ebcbe1abe8eebe327b96458d905c22",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/da9e5533acc8c01c61877b6613ddf40bdce9a8b4"
        },
        "date": 1788859466565,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 954973.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217338.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 818302.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 625175,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d66985dfc19d7e0a2ad278bfb29e8adec87f3adc",
          "message": "Merge pull request #322 from LoveDaisy/test/random-source-exact-assertion-audit\n\ntest: audit random sources behind exact assertions, and refill the lost closed-form fuzz",
          "timestamp": "2026-09-08T19:05:43+08:00",
          "tree_id": "559a5d866b5f56b5751d9d1be56655c2412ea1a2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d66985dfc19d7e0a2ad278bfb29e8adec87f3adc"
        },
        "date": 1788866202924,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 871865.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1219755.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 820621,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 806711.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5d738bc07fade93832384583fd5655dda496ae4",
          "message": "Merge pull request #321 from LoveDaisy/ci/organization-and-windows-testing\n\nci(windows): route MSVC compilation through sccache",
          "timestamp": "2026-09-08T20:34:51+08:00",
          "tree_id": "8cbe8c8f0a06164c7e6f448e8e1b9b8f2b5aeff3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5d738bc07fade93832384583fd5655dda496ae4"
        },
        "date": 1788871602788,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 853838.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217164.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 821670.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 702479.9,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5dab8ac5279acc30e74281d28ec9852f7091260f",
          "message": "Merge pull request #323 from LoveDaisy/feat/annotation-label-line-independence\n\nfeat(config): give the three grid families a line switch of their own",
          "timestamp": "2026-09-08T21:51:50+08:00",
          "tree_id": "17e102e9476ace17c070d7edc5d9863355c01883",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5dab8ac5279acc30e74281d28ec9852f7091260f"
        },
        "date": 1788876188885,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 933334.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1211581.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 759839.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 626731.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e06d6f8ee003f53159f4265e0a60478c3912298f",
          "message": "Merge pull request #324 from LoveDaisy/perf/cli-render-poll-floor\n\nperf(cli): poll completion before sleeping, so a render is not floored at 1s",
          "timestamp": "2026-09-08T22:52:18+08:00",
          "tree_id": "0e0c25fa9ce2a27fc0554ad989a39b2c64786891",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e06d6f8ee003f53159f4265e0a60478c3912298f"
        },
        "date": 1788879785389,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1048533.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1216159.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 823560.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 667733.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dfb3f72cde1303813ed87b0403da0bf2d5264b86",
          "message": "Merge pull request #325 from LoveDaisy/fix/user-run-vs-backpressure-gate\n\nfix(gui): exempt a user-initiated Run from the commit backpressure gate",
          "timestamp": "2026-09-08T23:07:48+08:00",
          "tree_id": "3e55872359ee24f0c6224f4e627e5dd6964a1d7d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfb3f72cde1303813ed87b0403da0bf2d5264b86"
        },
        "date": 1788880729931,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 968328.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1214975.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 826837.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 626662.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61afdda60a81f3800fe9d39d0f1790efbed2eb82",
          "message": "Merge pull request #326 from LoveDaisy/ci/cuda-test-tu-compile-coverage\n\nci: compile the CUDA test TUs (close the CUDA×BUILD_TEST empty intersection)",
          "timestamp": "2026-09-09T09:01:05+08:00",
          "tree_id": "d8015a4a11ce1b395d085be4867bade1cd75f056",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61afdda60a81f3800fe9d39d0f1790efbed2eb82"
        },
        "date": 1788916300005,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1000537.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1221757.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 820672.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 809410.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5d04ed2b510cf8da1331d69d8a0ec9d064e8688a",
          "message": "Merge pull request #328 from LoveDaisy/feat/gui-import-capability-boundary\n\nfeat(gui): warn on intentionally unsupported capabilities when importing core/CLI configs",
          "timestamp": "2026-09-09T11:28:47+08:00",
          "tree_id": "5fb7bd048fbeeb0720b37f54dc406c24b4f696f7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5d04ed2b510cf8da1331d69d8a0ec9d064e8688a"
        },
        "date": 1788925161823,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 929318.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1216021.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 822254.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 667280.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fe4c0778ad67356b7107c49b9f2d1758751cbab0",
          "message": "Merge pull request #329 from LoveDaisy/fix/raypath-load-path-syntax-gate\n\nfix(gui): reject malformed raypath summand rows on the .lmc load path",
          "timestamp": "2026-09-09T11:50:12+08:00",
          "tree_id": "85ae4abe8324f97a506977a0b6b5e2f1ee1a6194",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fe4c0778ad67356b7107c49b9f2d1758751cbab0"
        },
        "date": 1788926445499,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 925137.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1220731.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 827201.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 699231.4,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61199dc71bb8afa824f182ecb361c1765212e2ba",
          "message": "Merge pull request #330 from LoveDaisy/build/cpm-cache-shared-default\n\nbuild(cpm): default the dependency-source cache to a machine-level directory",
          "timestamp": "2026-09-09T12:28:59+08:00",
          "tree_id": "401ce6afc69f30c5242ddcb0e9b1c84c265275ad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61199dc71bb8afa824f182ecb361c1765212e2ba"
        },
        "date": 1788928787906,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 847754.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1219430.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 826846.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 806459.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "52b769f8831d3826607f24929ab7130e9dc62d1e",
          "message": "Merge pull request #331 from LoveDaisy/refactor/field-set-sentinel-proxy\n\nrefactor(config): guard RenderConfig's field set by member count, not sizeof",
          "timestamp": "2026-09-09T13:13:40+08:00",
          "tree_id": "ea00f972f5073c2d5a34b75fbe200d19d12f07b6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/52b769f8831d3826607f24929ab7130e9dc62d1e"
        },
        "date": 1788931513181,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1082517.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217969.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 818696.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 673174.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "96d644a21248a0968f4866679a3de372c1610833",
          "message": "Merge pull request #334 from LoveDaisy/feat/bg-image-color-picker\n\nfeat(gui): sample Sky Color off the background photo with an eyedropper",
          "timestamp": "2026-09-10T01:16:27+08:00",
          "tree_id": "dacea434bc3720c060ce99ea1e1f7083321478c0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/96d644a21248a0968f4866679a3de372c1610833"
        },
        "date": 1788974904755,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1014215.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1215012.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 814779.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 812055.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "258d9d34fde41b6c3c29d2818e91a4a20c13f2af",
          "message": "Merge pull request #335 from LoveDaisy/ci/drop-unused-vendor-apt-source\n\nci: stop depending on a vendor apt source nothing here installs from",
          "timestamp": "2026-09-10T02:19:42+08:00",
          "tree_id": "0ec8ef1453363cf2a4e9bac9403be312d6a3d461",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/258d9d34fde41b6c3c29d2818e91a4a20c13f2af"
        },
        "date": 1788978634096,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 771436.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1220570.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 826659,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 674290.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "712eb886076cecd28b4dedc683f8255351558cb5",
          "message": "Merge pull request #333 from LoveDaisy/ci/cache-budget\n\nci(cache): budget the actions/cache quota — fix three prefix-shadowed keys, add ccache to the critical-path leg",
          "timestamp": "2026-09-10T02:35:58+08:00",
          "tree_id": "c53075d893ac829084799f406bdd8c280a195292",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/712eb886076cecd28b4dedc683f8255351558cb5"
        },
        "date": 1788979617888,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1057995.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217168.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 895353,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 627227.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "491b117b9a07cdf85de8099529f7811e686abf1e",
          "message": "Merge pull request #336 from LoveDaisy/feat/miller-index-and-wedge-presets\n\nfix(gui,core): give the Miller-index wedge conversion one owner, and correct the presets it was never checked against",
          "timestamp": "2026-09-10T04:26:14+08:00",
          "tree_id": "4e290141061128a452482994544759c4c4475a08",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/491b117b9a07cdf85de8099529f7811e686abf1e"
        },
        "date": 1788986313728,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1000101.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217212.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 784590.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 666161.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a4c526050e74287deacd4473631926deabf13a9",
          "message": "Merge pull request #337 from LoveDaisy/feat/print-mode-subtractive-ink\n\nfeat(render,gui): add a print tone that lays ink on paper instead of adding light to sky",
          "timestamp": "2026-09-10T09:06:10+08:00",
          "tree_id": "7d1fa0a3596ea0279c8d776fa7d4f0baaa8feab6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a4c526050e74287deacd4473631926deabf13a9"
        },
        "date": 1789003113626,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 839561.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217551.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 819537.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 671185.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ae46f7c283910d4fbb4e7a0f2949aef02e5f87df",
          "message": "Merge pull request #338 from LoveDaisy/feat/gui-display-rendering-regroup\n\nfix(gui): regroup the Display Rendering rows and pair the ground swatch with the mode",
          "timestamp": "2026-09-10T14:08:18+08:00",
          "tree_id": "eeaec146eaa0226974bb83e95e4e1b36f34970a9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ae46f7c283910d4fbb4e7a0f2949aef02e5f87df"
        },
        "date": 1789021106000,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1016691.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1220236.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 817266.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 660118.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3ad57411dc16f516b6785967efaba5266c88e7b8",
          "message": "Merge pull request #339 from LoveDaisy/feat/test-capi-lib\n\ntest: liblumice_testapi, a test-only export surface beside the product C API",
          "timestamp": "2026-09-10T16:59:47+08:00",
          "tree_id": "91bfa648700adc1c02137e2bab552ca4271f3417",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3ad57411dc16f516b6785967efaba5266c88e7b8"
        },
        "date": 1789031375132,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 798856.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1219764.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 817944.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 666878.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "724fa7cff5ff0cc832f33d45b08e1bf3d4536f40",
          "message": "Merge pull request #342 from LoveDaisy/feat/annotation-lines-shader-anchors-api\n\ngui: auxiliary lines track the camera every frame again; anchors-only annotation API (v4.28)",
          "timestamp": "2026-09-10T17:18:09+08:00",
          "tree_id": "7f9537581734b6e612b1d500271e44df92102186",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/724fa7cff5ff0cc832f33d45b08e1bf3d4536f40"
        },
        "date": 1789032511943,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 856421.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1215198,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 756353.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 808847.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d57f132dc2f546ad001a215fb28d6c917bddabf4",
          "message": "Merge pull request #340 from LoveDaisy/docs/working-discipline-hardening\n\ndocs+hooks: harden two working-discipline rules into criteria and a commit gate",
          "timestamp": "2026-09-10T18:06:39+08:00",
          "tree_id": "1e03b6ab5fd5688bd565295581883fc4c9690d85",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d57f132dc2f546ad001a215fb28d6c917bddabf4"
        },
        "date": 1789035355617,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 896997.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1211723.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 822315.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 633852.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "37b141504a40cc0be937f0d5bf071fef24759171",
          "message": "Merge pull request #341 from LoveDaisy/test/defaults-panel-refs-reshoot\n\ntest(gui): pin the wedge add row in every preset scene, and re-shoot the two that were not",
          "timestamp": "2026-09-10T18:49:54+08:00",
          "tree_id": "42fe951d730ad0cd31b0d30b5c6fc14fa0ec2dc1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37b141504a40cc0be937f0d5bf071fef24759171"
        },
        "date": 1789038031511,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 877212.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1210583.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 819254.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 810913.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f860acc46471dee85057842a8611895cea64b88d",
          "message": "Merge pull request #343 from LoveDaisy/feat/gui-print-mode-label-ink\n\nfix(gui): draw overlay label text as ink under the print tone",
          "timestamp": "2026-09-10T20:57:53+08:00",
          "tree_id": "f3daf403c69c5adec882772328d1a77ec96a9215",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f860acc46471dee85057842a8611895cea64b88d"
        },
        "date": 1789045695822,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 763603.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1212271.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 955404.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 674721.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dc76b64939e2b7e7bb319dee15905da1c73ee7fa",
          "message": "Merge pull request #344 from LoveDaisy/feat/image-comparison-metric-by-layer\n\ntest: give each image comparison a ruler that matches its layer (pixel ruler, lines-only parity, block-mean PSNR)",
          "timestamp": "2026-09-11T01:37:19+08:00",
          "tree_id": "41b16f85c1532f746a24d73f00d8388af7f2060a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dc76b64939e2b7e7bb319dee15905da1c73ee7fa"
        },
        "date": 1789062631306,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 814269.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1213530.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 817358.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 692646.5,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "38aff9c6a97f3fdcca9801ffeb6e1dcecf4be998",
          "message": "Merge pull request #345 from LoveDaisy/chore/release-4.5.1\n\nrelease: cut 4.5.1, and make the release a per-version backfill chore",
          "timestamp": "2026-09-11T08:06:39+08:00",
          "tree_id": "c20bb077289e78ac31076d801efb94c50cab02ad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/38aff9c6a97f3fdcca9801ffeb6e1dcecf4be998"
        },
        "date": 1789085819488,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 776743.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1209815.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 819284.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 801751.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "142e29e615d7a573006eabf610b33948e5584c98",
          "message": "Merge pull request #346 from LoveDaisy/feat/hardware-perf-distribution\n\nbuild/release: ship ISA- and GPU-matched binaries behind CPUID launchers (x86-64-v4 Linux, x86-64-v3 clang-cl Windows, sm_120 fatbin)",
          "timestamp": "2026-09-11T20:51:32+08:00",
          "tree_id": "f05bd3485353b55d626d7d9fe93091774693bb97",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/142e29e615d7a573006eabf610b33948e5584c98"
        },
        "date": 1789131614038,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 998804.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1214310.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 751824.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 667423.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4fdfe61cc4b1326f60a924a669133f7c6311023d",
          "message": "Merge pull request #347 from LoveDaisy/feat/raypath-analysis-panel\n\nfeat: raypath analysis panel — dedicated non-rendering pass, ranked by chain",
          "timestamp": "2026-09-12T16:27:12+08:00",
          "tree_id": "071aa48f973504cccddec5a5f2199596e22adfc9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4fdfe61cc4b1326f60a924a669133f7c6311023d"
        },
        "date": 1789202421547,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1030197.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1189140.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 738088.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 667082.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "70eb8f5fad44336cf2b57da4d314a9aced4c8224",
          "message": "Merge pull request #349 from LoveDaisy/feat/raypath-analysis-followups\n\nRaypath analysis follow-ups: fixed-seed reproducibility, session-kind rebuild predicate, joiner glyphs, debt sweep",
          "timestamp": "2026-09-12T23:46:53+08:00",
          "tree_id": "1570515bb91fdd1a4610a3ac00dc77a6ae1f7cca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/70eb8f5fad44336cf2b57da4d314a9aced4c8224"
        },
        "date": 1789228786544,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1083377.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1180853.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1232767.7,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 663385.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c106292be4122b60092a9666131dde843f639b15",
          "message": "Merge pull request #348 from LoveDaisy/feat/crystal-ray-allocation\n\nfeat(core): adaptive ray allocation across crystal entries (scene.ray_allocation)",
          "timestamp": "2026-09-13T04:19:28+08:00",
          "tree_id": "82448d3731df653e760db492e5a8794a13dc6f0c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c106292be4122b60092a9666131dde843f639b15"
        },
        "date": 1789245355985,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1070986.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1188828.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 801067.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 626023.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "88e0fbf6864b1d95ba7e19c4d6660fb8c15c1f4c",
          "message": "Merge pull request #350 from LoveDaisy/chore/install-manual-refresh-and-review-minors\n\nchore: refresh the install manual, land the metric-by-layer review minors, report wrong-size anchor planes once",
          "timestamp": "2026-09-13T04:52:15+08:00",
          "tree_id": "94bf3da366cd365b6946ce53b5309cd6df95c36e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/88e0fbf6864b1d95ba7e19c4d6660fb8c15c1f4c"
        },
        "date": 1789247023205,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1116608.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1181067.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 801885.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 626387.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d5c230764f43779ffb32bc75e454ec07f2159a03",
          "message": "Merge pull request #351 from LoveDaisy/fix/exposure-mode-combo-fixed-separation\n\ntest(gui): prove exposure-mode separation with an intensity probe, not a seed-dependent gap",
          "timestamp": "2026-09-13T05:13:06+08:00",
          "tree_id": "8da3f1ccd7d22ef20d6fff6dff9a5c615f8e013d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d5c230764f43779ffb32bc75e454ec07f2159a03"
        },
        "date": 1789248086988,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1101595.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1181361.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 742006.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 997131.7,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a68880e398137ef91895edbb6d4ff0c997923e4",
          "message": "Merge pull request #352 from LoveDaisy/chore/regen-refs-deterministic-single-shot\n\nchore(regen-refs): shoot deterministic groups once, share runs across groups, refuse stale-base reshoots",
          "timestamp": "2026-09-13T05:31:09+08:00",
          "tree_id": "5e9ceaec07873c2b9154175e6d957784999b8820",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a68880e398137ef91895edbb6d4ff0c997923e4"
        },
        "date": 1789249342472,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 999683.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1181956.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 802031.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 675892.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "365409776ad9761a5ebf3402cf9cde48f573e9d8",
          "message": "Merge pull request #353 from LoveDaisy/fix/render-consumer-label-flake-root-cause\n\nfix(test): root-cause the RenderConsumerLabel flake — an uninitialized SunParam azimuth",
          "timestamp": "2026-09-13T05:47:24+08:00",
          "tree_id": "2a620f7705b88d7f56d29a1cf923e190e358fd9e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/365409776ad9761a5ebf3402cf9cde48f573e9d8"
        },
        "date": 1789250200819,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1119652.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1187489.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 865896.2,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 812746.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "35235a01c6962917a905abe105b458ee0ba444ab",
          "message": "Merge pull request #354 from LoveDaisy/feat/ray-num-slider-100b-log-scale\n\nfeat(gui): Rays(M) slider spans 0.1..100 000 M on a kLog track, one domain for both rows",
          "timestamp": "2026-09-13T06:17:09+08:00",
          "tree_id": "8c31257305277f134a3473abfb64eca3bdbdbc3a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35235a01c6962917a905abe105b458ee0ba444ab"
        },
        "date": 1789252145956,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 859107.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1188364,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 808237,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1013551.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f0add0b4a676a2e20ab27c782e9e9b5a182aac5b",
          "message": "Merge pull request #355 from LoveDaisy/feat/cli-lens-and-grid-contract\n\nfeat(lens): the CLI/GUI lens contract — short-edge fov, defaults, focal length import, annotations at intensity 0",
          "timestamp": "2026-09-13T07:31:34+08:00",
          "tree_id": "14ed237826bf2e9217fc60dc8c0bd508aee5669a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f0add0b4a676a2e20ab27c782e9e9b5a182aac5b"
        },
        "date": 1789256495974,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 929375.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1186105,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 958150.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 661972.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "51fa59e29850545fd09b7f6041faaed4a4bea4cd",
          "message": "Merge pull request #356 from LoveDaisy/feat/cuda-hostgen-black-and-energy-accounting\n\nfix(cuda): host root-gen fallback renders again; landed weight reduced per warp so the energy ledger matches legacy",
          "timestamp": "2026-09-13T08:09:28+08:00",
          "tree_id": "5310d81311d369114cdde3eb97a22ec84b3b27c2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51fa59e29850545fd09b7f6041faaed4a4bea4cd"
        },
        "date": 1789258763979,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1045324.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1180404.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 813138.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 812113,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "22166140295c68e58e6375028394d4587a351c11",
          "message": "Merge pull request #357 from LoveDaisy/feat/view-center-angular-dist-grid\n\nfeat(annotation): view_dist — circles of constant angular distance from the optical axis, config → core → C API → GUI",
          "timestamp": "2026-09-13T12:02:01+08:00",
          "tree_id": "6875aee1957344381ca66a902bb0dc2ba20f8f02",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/22166140295c68e58e6375028394d4587a351c11"
        },
        "date": 1789273862949,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1240350.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1182444.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 802146.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1016951.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "682bf9aadbd77a61c2d7697ccd4b353bb70a06be",
          "message": "Merge pull request #358 from LoveDaisy/fix/equidistant-focal-length-factor-two\n\nfix(config): equidistant lens f→fov conversion was half the documented value",
          "timestamp": "2026-09-13T12:42:20+08:00",
          "tree_id": "ad66fbc16fa5d96511d2e7667a3348c6c69b86d5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/682bf9aadbd77a61c2d7697ccd4b353bb70a06be"
        },
        "date": 1789275329406,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 808629,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1180008.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 803160.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 665843.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b10d1bbc134afb64423fa842b7740de5a59934f2",
          "message": "Merge pull request #359 from LoveDaisy/feat/analysis-panel-polish\n\nfeat(gui): raypath analysis panel polish — first-picture gate, draw layer, geometry, thousands grouping, Export CSV",
          "timestamp": "2026-09-13T14:55:21+08:00",
          "tree_id": "ffbfcccfae868fd3b9504d3d98acd998d41a1933",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b10d1bbc134afb64423fa842b7740de5a59934f2"
        },
        "date": 1789283178026,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1239098.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1180647.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 880752.2,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 623845.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "90d23cafd492e0f65663df5a681c439b0fa09f35",
          "message": "Merge pull request #360 from LoveDaisy/feat/analysis-standing-cpu-pool\n\nfeat(server): standing CPU analysis pool on the GPU route, woken by session kind",
          "timestamp": "2026-09-13T15:15:21+08:00",
          "tree_id": "0e81a696990d2128a156faf657717d6b2838d130",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/90d23cafd492e0f65663df5a681c439b0fa09f35"
        },
        "date": 1789284492874,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 865014,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1184775.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 949906.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 673336.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      }
    ],
    "Parallel Efficiency": [
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1a021d0025f312751971b24b6417dc1662c14555",
          "message": "Merge pull request #263 from LoveDaisy/chore/doc-stale-state-claims\n\nMake the docs and comments say what the code actually does now",
          "timestamp": "2026-08-11T12:09:28+08:00",
          "tree_id": "1cb6721b617dd3dce5b8306763e1828b8317f689",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1a021d0025f312751971b24b6417dc1662c14555"
        },
        "date": 1786422068543,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 76.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "39bb35c6eee19cccafa61c3f38f67c65cbfec9b6",
          "message": "Merge pull request #264 from LoveDaisy/chore/test-premise-expiry-and-gate-justification\n\ntest: retire five dead observation channels and one lying marker",
          "timestamp": "2026-08-11T20:15:43+08:00",
          "tree_id": "d2f97ffb2e4eada1402851069e948df82831d91b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/39bb35c6eee19cccafa61c3f38f67c65cbfec9b6"
        },
        "date": 1786451270108,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 62.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "a1957e028279c05871bc50cbd1001cacb1aa2ee8",
          "message": "docs: make a completeness claim carry the same burden of proof as adding code\n\n\"Covers all 20 panels with zero omissions\" reads as an achievement and passes\nreview unchallenged; \"this class is not worth covering\" has to be argued for.\nThat asymmetry is the default state rather than anyone's choice, so completeness\nwins every conflict without a single person advocating for it -- including\nconflicts against the budget the same task committed to.\n\nPR #261 is the measured instance: a pre-committed target of -30% de-commented\ntest lines (baseline 21,336, pinned by two independent counters with 52/52 files\nzero diff) landed at -8.7%, and the coverage backfill demanded by \"20 panels,\nzero omissions\" accounts for roughly a third of the miss. Escape-defect density\nover those same files had already been measured and spans 8x; the equal-weight-\nper-panel split discarded that measurement.\n\nThe rule asks for the justification, not the reduction. Whether a leaner suite\nwould have let more defects escape is a counterfactual and untestable, so this\nis explicitly not a mandate to cut -- only a requirement that an equal-weight\npartition state its reason when a per-member value measure is available.\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>",
          "timestamp": "2026-08-12T08:19:41+08:00",
          "tree_id": "45eab6bd9bc7f45bb51575ce022fbe40ff3e9e32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a1957e028279c05871bc50cbd1001cacb1aa2ee8"
        },
        "date": 1786494694783,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 67.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3b06a512d9e1c500bf53c2ae03ebc98e78a8ee6d",
          "message": "Merge pull request #267 from LoveDaisy/chore/perf-doc-machine-provenance\n\ndocs: 远程验证文档按「角色 / 主机绑定」分层，并写入新参照机 recipe",
          "timestamp": "2026-08-12T15:34:26+08:00",
          "tree_id": "622acaacaf2c20558ea2765a841267f06017335f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3b06a512d9e1c500bf53c2ae03ebc98e78a8ee6d"
        },
        "date": 1786520737531,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 87.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "660cb36f3211d40db009ebe4cb974d819900051d",
          "message": "Merge pull request #268 from LoveDaisy/task/msvc-portability-test-env-helper\n\nfix(test): 把 setenv/unsetenv 的 #ifdef 收敛成一个有名字的 helper",
          "timestamp": "2026-08-12T20:45:55+08:00",
          "tree_id": "a60dffae6ac4ce8bfff90f14b0a1c3d1815c8fe7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/660cb36f3211d40db009ebe4cb974d819900051d"
        },
        "date": 1786539480463,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8684aa9e5d304592bc3fe6a782307b7a9da8c704",
          "message": "Merge pull request #269 from LoveDaisy/task/win-static-crt-cmp0091\n\nfix(build): 让 CMP0091 真正生效，Windows 发布产物链接静态 CRT",
          "timestamp": "2026-08-13T17:00:34+08:00",
          "tree_id": "9e6e68dabd87f16fcb817b3f0ef3dcfef9246ada",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8684aa9e5d304592bc3fe6a782307b7a9da8c704"
        },
        "date": 1786612352169,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 92.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 97.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7da5553043fc957ef127227fd09c6bd2a434969a",
          "message": "Merge pull request #270 from LoveDaisy/docs/gui-blueprints\n\ndocs(gui): 落盘视觉语言与布局架构两份 GUI 蓝图",
          "timestamp": "2026-08-14T01:14:13+08:00",
          "tree_id": "d80b51f210e34913cdcdae277891c8370b111d51",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7da5553043fc957ef127227fd09c6bd2a434969a"
        },
        "date": 1786641957961,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 72.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "de2400d72e0c74815654b5c0798537d321c36342",
          "message": "Merge pull request #271 from LoveDaisy/task/gui-visual-language\n\nfeat(gui): 落地 GUI 视觉语言——单一 owner、比例字体、量化节奏、调色板与语义色",
          "timestamp": "2026-08-14T08:18:19+08:00",
          "tree_id": "e4dae219cff3b357fa69562f4581b9a13c1244c7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de2400d72e0c74815654b5c0798537d321c36342"
        },
        "date": 1786667413863,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e5a855759f2a1e535cf1ff56144d57a93472b4c1",
          "message": "Merge pull request #272 from LoveDaisy/feat/new-gui-layout\n\nfeat(gui): 新 GUI 布局——「文档 | 图像 | 运行」三区重组（集成分支）",
          "timestamp": "2026-08-18T13:27:49+08:00",
          "tree_id": "4d399707fc655846f8bae9083488c3c91c9ce3aa",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e5a855759f2a1e535cf1ff56144d57a93472b4c1"
        },
        "date": 1787031518736,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 97.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a66c523b420650ecb6f8abbe787e38f506ba4e3",
          "message": "Merge pull request #273 from LoveDaisy/feat/gui-form-refinement\n\nfeat(gui): 控件形态精修——宽度 token、PropertyRow 与排版秩序",
          "timestamp": "2026-08-19T02:57:24+08:00",
          "tree_id": "732853fea0235432d76a2465b680be58ccc15e51",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a66c523b420650ecb6f8abbe787e38f506ba4e3"
        },
        "date": 1787080145142,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 101.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "f1228505c4ed659c006158629ed8b4501eba7074",
          "message": "docs(gui-layout): 记录内测否决，蓝图从待办降为设计记录\n\nv4.4.2（老 shell）与 v4.4.2-new（新 shell）小范围内测对比后，几乎全部\n内测用户选择回到老 shell。main 回退到 PR #271：视觉语言层留下，形态层\n（PR #272 shell 重组 + PR #273 控件精修）退出，实现保存在分支\nfeat/new-gui-layout 与 tag/release v4.4.2-new。\n\n三处改动都是为了让下一个读者不把已被否决的方向当成在途的待办：\n\n1. gui-layout-architecture.md 顶部状态改写 + 新增 §8。记下三件事：反馈\n   粒度未知（聚合结论没区分拒的是形态还是外观，故保留视觉语言层既不由\n   它支持也不被它否定，下一步取证是老 shell 上单发视觉语言层做窄 A/B）；\n   方法层教训（原型验收与 owner 上手两道闸共享同一盲区——都在问「形态\n   本身好不好」，没问「熟练用户是否愿意换」，而后者才是内测在问的）；\n   以及没有被否决的部分（§0 诊断对今天的老 shell 仍为真，§5 六条被推翻\n   形态不恢复候选资格）。\n\n2. gui-visual-language.md 更正时态。该文 §4 定案随 PR #271 留在 main 上，\n   但文中多处把 docking 迁移写成在途的事，回退后不再成立，一律改读作\n   「将来任何一次面板重排」。同时补回 §7 正文字体的收口——字体定案\n   （Roboto Medium 15 构建期嵌入）随 PR #271 落地，而写下这条收口的文档\n   改动落在 PR #273 里，被本次回退一并带走，留下文档说「未定案」而代码\n   已定案的漂移。\n\n3. AGENTS.md 两条索引同步。索引是这两份文档唯一的必经检索入口，否决\n   记录只写在文档里而不写在索引上，等于没写。",
          "timestamp": "2026-08-26T10:15:35+08:00",
          "tree_id": "0508524749ee48fed4c9dbf374750ff51a0579f3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1228505c4ed659c006158629ed8b4501eba7074"
        },
        "date": 1787711364757,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "840bc16a46aa7fc6e6265bd34609af330a10af64",
          "message": "docs(gui-layout): 形态层的锚点从 v4.4.2-new 标签改为分支 + commit\n\n内测反馈已到手，v4.4.2-new 标签与 release 随之删除（留着它就是把已被\n否决的界面挂在 Latest release 上发给外部用户）。但 §8 与 AGENTS.md 索引\n都拿这个标签当「形态层保存在哪里」的锚点，标签一删锚点就悬空。\n\n改为锚在分支 feat/new-gui-layout 与 commit 7a66c523——commit hash 是\n永久锚点，分支是可读入口。§8 同时留一句说明标签删除的原因和重出该构建\n的办法（从该分支重新打标签），免得下一个读者以为构建丢了。",
          "timestamp": "2026-08-26T10:28:26+08:00",
          "tree_id": "c793fa450b5d26416c5676defa6dd7ccb9d2badc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/840bc16a46aa7fc6e6265bd34609af330a10af64"
        },
        "date": 1787712084349,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "9be4b99e4907e79992121a4e19f6dd742ba03dea",
          "message": "docs(gui-layout): 原型取证锚点随原型分支一同退役\n\n三个 spike 分支（gui-layout-prototype / gui-visual-language / imgui-docking）\n从未推送、只存在于本地，随新布局方向被内测否决一并删除。布局蓝图开头\n把其中 gui-layout-prototype 及三个 commit 写作「取证锚点」，分支一删这行\n就指不到任何东西。\n\n改为如实说明：取证现场已不存在，§1–§5 此后是已记录的判断而非可重新核验\n的断言；要看那一版形态实际长什么样，去 feat/new-gui-layout——同一形态的\n完整实现，完成度高于原型，只不含 §5 那六条从未被实现的候选。\n\n§5 开头补一句界定：六条结论不因原型删除而撤销，重提的一方承担举证责任。\n顺带修一处漂移：视觉语言 §4.1 仍写着「具体字体尚未定案」，而 §7 的收口\n和 main 上的代码都已是 Roboto Medium 15。",
          "timestamp": "2026-08-26T11:34:40+08:00",
          "tree_id": "f4517359a5286de30e0bd6f6f01ac44f5b0b416e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9be4b99e4907e79992121a4e19f6dd742ba03dea"
        },
        "date": 1787715961619,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 72.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "41354876b363944c8882dba3c46014eb797382aa",
          "message": "Merge pull request #274 from LoveDaisy/task/bg-image-filtering\n\nfix(gui): 底图纹理改用 mipmap + trilinear，修缩小显示时的欠采样混叠",
          "timestamp": "2026-08-26T20:00:46+08:00",
          "tree_id": "a5fbb679ee0ffcf9c28c9aec23ea64e6663e04a8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/41354876b363944c8882dba3c46014eb797382aa"
        },
        "date": 1787746342187,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 93,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f17b841b725042bd28408b6db3fd1da5cfc7fa8f",
          "message": "Merge pull request #275 from LoveDaisy/task/bg-image-transform\n\nfeat(gui): 底图可平移缩放，让裁剪过的照片能与仿真结果对齐",
          "timestamp": "2026-08-26T20:20:53+08:00",
          "tree_id": "24754ad01298b2e39006a4bc1d728369562e429c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f17b841b725042bd28408b6db3fd1da5cfc7fa8f"
        },
        "date": 1787747563685,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e23d669872bbfb1762a95659960d412d793c55e9",
          "message": "Merge pull request #276 from LoveDaisy/task/crystal-enable-toggle\n\nfeat(gui): 晶体卡新增「参与仿真」toggle，替代把权重拖到 0",
          "timestamp": "2026-08-26T20:39:00+08:00",
          "tree_id": "4b0f2abbc5fcc90f68d6ae6281bfef446c82cc03",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e23d669872bbfb1762a95659960d412d793c55e9"
        },
        "date": 1787748633312,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "423ff22e33a81fc6521fb8da4f55037e88e7ac93",
          "message": "docs(gui-layout): 内测反馈的细粒度到手，改掉三处已被它推翻的记录\n\n上一轮记录写于反馈只有聚合结论时，有三处现在是错的，且都写在下一个读者\n必经的位置上。\n\n1. 「反馈粒度未知」作废。细粒度反馈是：配色被接受（用户对配色的接受范围\n   很宽），被拒的是形态，理由具体——不如老 plain 布局一眼看到所有信息，\n   典型操作「同时快速调整冰晶与太阳高度」要多点好几步、来回切换不便。\n   于是原计划的「老 shell 上单发视觉语言层做窄 A/B」不必做了：那道取证是\n   为了问出粒度，粒度已经有了。\n\n   机制不是打磨不足，是 master-detail 的结构性代价：老 shell 左栏晶体卡与\n   右栏 Scene（含太阳）永久同时在屏，导航成本为零；新形态里太阳是检视器的\n   一个 page，晶体是同一检视器的另一个 page，一次只看得见一个对象。\n\n2. 方法层教训改写。原先写作「两道闸共享盲区＝没问熟练用户是否愿意换，\n   验收链必须含一条能测迁移成本的证据」——这个说法经不起推敲：本地开发阶段\n   必然只能问「这个形态好不好」，必然要发版才拿得到用户反馈，那不是一道\n   本可设而没设的闸，而是结构性事实；发版、拿反馈、便宜回退、分支留存，\n   这个环当时是通的。真正的偏差是验收问错了量——两道闸问的都是「形态本身\n   好不好用」，而用户答的是两个本地就能机械量出来却从没被量过的数：常见\n   任务的操作步数，以及一屏同时可见的字段集合。\n\n3. §0 第一条补一个维度。§0 骂老 shell「分割轴任意」属实，但「任意」不等于\n   「差」：那条任意的轴恰好让最常一起调的两组永久同屏，而 §0 从未度量过\n   同屏可见性这一维，用户却只在这一维上表了态。因此追加一条硬约束——将来\n   任何一次重排，最常一起调的字段组必须保持同屏可见——地位等同 §5 那六条\n   被推翻形态。\n\ngui-visual-language.md 顶部同步：那条「不构成外观已被接受的证据」作废，但\n边界要写清楚，被问到的只有配色，§4 其余条目仍只是「没有被反对」。\nAGENTS.md 两条索引一并同步——索引是这两份文档唯一的必经检索入口。\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>",
          "timestamp": "2026-08-26T20:49:35+08:00",
          "tree_id": "6b66463dd3987722c0987a1f07a416d5332cca1b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/423ff22e33a81fc6521fb8da4f55037e88e7ac93"
        },
        "date": 1787749585132,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "eea0a7b0268762f9590f8d50912a3662a80e6936",
          "message": "Merge pull request #277 from LoveDaisy/task/gui-overlay-table\n\nfeat(gui): Overlay 辅助线组改为 6 列表格形态",
          "timestamp": "2026-08-29T12:19:58+08:00",
          "tree_id": "7e912e59f4173ec789186af3f24b4e7329ddef8d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/eea0a7b0268762f9590f8d50912a3662a80e6936"
        },
        "date": 1787977873710,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ff02ed3a487ac6e46dd2720351057ed2314aa60b",
          "message": "Merge pull request #278 from LoveDaisy/task/full-sphere-roll-flip\n\nfix(core+gui): 全球面快路径补 roll 旋转对称条件，修滑条端点浮点漂移导致的采样路静默切换",
          "timestamp": "2026-08-29T13:20:57+08:00",
          "tree_id": "12a2380afc7b07d8b15f664fa3f2bf288d3e7f1e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ff02ed3a487ac6e46dd2720351057ed2314aa60b"
        },
        "date": 1787981571254,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6320bbab9d388df3b07a528374e61df82d920a6b",
          "message": "Merge pull request #279 from LoveDaisy/task/gui-theme-color-closure\n\nrefactor(gui): 颜色收口——色槽补齐 58/58 + 调用点裸字面量逐处 disposition",
          "timestamp": "2026-08-29T15:06:01+08:00",
          "tree_id": "082d1f2e575f42837f85a5f180ed8d57c1ca7176",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6320bbab9d388df3b07a528374e61df82d920a6b"
        },
        "date": 1787987827140,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 100.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "994823ec3a42f7c5fb27150249b58b30ac4f6336",
          "message": "Merge pull request #280 from LoveDaisy/task/overlay-table-acceptance-fixes\n\nfix(gui): Overlay 表格人工验收三条修复",
          "timestamp": "2026-08-29T22:17:55+08:00",
          "tree_id": "6d0628441baf2aabfb41489d69d226d3905d11b0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/994823ec3a42f7c5fb27150249b58b30ac4f6336"
        },
        "date": 1788013781307,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 91.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "26e730907f491d40869033a4ce67ac66edc88f8a",
          "message": "Merge pull request #281 from LoveDaisy/task/gui-label-column-gap-alignment\n\nfix(gui): 行末标签列左缘对齐 + 间距收敛为单一 owner",
          "timestamp": "2026-08-30T11:28:21+08:00",
          "tree_id": "51266763b2f5e9a7423d99f0a75aaf7d05257f11",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/26e730907f491d40869033a4ce67ac66edc88f8a"
        },
        "date": 1788061026049,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 89.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 92.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "37658798751c385f405d25527fc3226fb807ff08",
          "message": "Merge pull request #282 from LoveDaisy/task/gui-entry-card-layout-and-crystal-identity\n\n晶体卡片 layout 重排 + 晶体身份可寻址 + Colors 面板编号/失效态",
          "timestamp": "2026-08-30T13:34:44+08:00",
          "tree_id": "23118e5367b29ce0c82bda8278360f565c79c8b8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37658798751c385f405d25527fc3226fb807ff08"
        },
        "date": 1788068777425,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "d63befda26e1912dcf6d6d8cc67efd7f34eb7ac2",
          "message": "bump patch version for release",
          "timestamp": "2026-08-30T17:17:40+08:00",
          "tree_id": "ae2567feaaad662c625ed888e28482837014d4c8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d63befda26e1912dcf6d6d8cc67efd7f34eb7ac2"
        },
        "date": 1788082223016,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 66.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "53299d835046ba74d6f2897c4a1566993368a6bf",
          "message": "Merge pull request #283 from LoveDaisy/task/gui-fisheye-lens-border\n\nGUI: 鱼眼镜头有效区边框辅助线",
          "timestamp": "2026-08-30T22:06:58+08:00",
          "tree_id": "2ba992405cad4908ee5cc797c9404a38a110ff48",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/53299d835046ba74d6f2897c4a1566993368a6bf"
        },
        "date": 1788099498760,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "68f3be6a4fc6353c202b92f6538e7b9c3d5d1100",
          "message": "Merge pull request #284 from LoveDaisy/task/retire-comma-raypath-separator\n\n退役 raypath 逗号连接符：静默算错改为指名改法的拒绝 + 加载期迁移",
          "timestamp": "2026-08-30T22:29:34+08:00",
          "tree_id": "e9cc435a031fe70e00ae4ab2e1c43c075f39eb24",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/68f3be6a4fc6353c202b92f6538e7b9c3d5d1100"
        },
        "date": 1788100793551,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c798112109aa5fb9e2f85564fd0f9bfb59e0259",
          "message": "Merge pull request #285 from LoveDaisy/task/user-defaults-schema-version\n\n给 user_defaults.json 盖上独立的 schema 版本戳（只记录，不设闸，不迁移）",
          "timestamp": "2026-08-31T08:46:01+08:00",
          "tree_id": "c6063d0d04375864505dbcfd05c8f5eba511d092",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6c798112109aa5fb9e2f85564fd0f9bfb59e0259"
        },
        "date": 1788137861222,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "034ab22193bfe8cf4e9efb4aac1599e82c5e3308",
          "message": "Merge pull request #286 from LoveDaisy/feat/adjustable-background-color\n\n可调背景颜色：GUI/CLI 五路一致 + core 定义域掩码 + 注记层处置",
          "timestamp": "2026-08-31T12:48:11+08:00",
          "tree_id": "465ab3c2e3bc53104f4df724fba2c95cd70a2d2c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/034ab22193bfe8cf4e9efb4aac1599e82c5e3308"
        },
        "date": 1788152410805,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 97.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8054154bedaaf1ce926cd9c7d44a6c1c548cc115",
          "message": "Merge pull request #287 from LoveDaisy/feat/absolute-ev\n\nfeat: 绝对 EV —— cross-simulation 可比的曝光尺度",
          "timestamp": "2026-08-31T15:03:03+08:00",
          "tree_id": "e66dfd46e67488974ac2ab7f738dc49b2e56328a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8054154bedaaf1ce926cd9c7d44a6c1c548cc115"
        },
        "date": 1788160555461,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5360f028303b6963e69eb26ba999c203a3f3018e",
          "message": "Merge pull request #288 from LoveDaisy/feat/cli-gui-render-parity\n\n让导出的 config 诚实描述用户所见 + 建 CLI↔GUI 出图对照闸",
          "timestamp": "2026-09-01T10:20:29+08:00",
          "tree_id": "1d4a30db33e4386ab9a58ed55adc7521892bc4bb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5360f028303b6963e69eb26ba999c203a3f3018e"
        },
        "date": 1788230002198,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "93c163fe3b8be8017bff712da303b6afa9ba8c03",
          "message": "Merge pull request #289 from LoveDaisy/task/lens-json-names-oob\n\nfix(gui): 修 kLensTypeJsonNames 越界读（用户可达崩溃）",
          "timestamp": "2026-09-01T11:38:10+08:00",
          "tree_id": "386dbbee06c0b18a0469f3dd58b480cc1f8c7e05",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/93c163fe3b8be8017bff712da303b6afa9ba8c03"
        },
        "date": 1788234599814,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9faf145b0edde6af078573ca8726d7ce246411df",
          "message": "Merge pull request #290 from LoveDaisy/task/preview-solid-angle-jacobian\n\nfeat(gui): 预览 shader 补上目标镜头的相对照度，使非等面积投影下 GUI 与 CLI 可逐像素比",
          "timestamp": "2026-09-01T13:00:21+08:00",
          "tree_id": "89c8de38143255c6034f17752c7461109dd74f23",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9faf145b0edde6af078573ca8726d7ce246411df"
        },
        "date": 1788239569242,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "03e21b4dd2baadc1325958ed61d1293e3d6434c9",
          "message": "Merge pull request #291 from LoveDaisy/feat/fisheye-domain-widening\n\nfeat(core): 单镜头鱼眼定义域按 lens 放宽到 θ≤180，与 GUI 对齐",
          "timestamp": "2026-09-01T15:17:01+08:00",
          "tree_id": "db0ee29334ec16918d95c3230818b73c6aa64f80",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/03e21b4dd2baadc1325958ed61d1293e3d6434c9"
        },
        "date": 1788247630723,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a6034051812988c3e1fb7639296b5d3fbdbf8375",
          "message": "Merge pull request #292 from LoveDaisy/feat/core-annotation-layer\n\nfeat(core): 注解层补齐——辅助线与文字 label 收敛为 core 单一来源",
          "timestamp": "2026-09-02T04:45:19+08:00",
          "tree_id": "fe59f2f65430792483491f011bdddca70953ff32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a6034051812988c3e1fb7639296b5d3fbdbf8375"
        },
        "date": 1788296273403,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fc5de377009120dd26063703a16f31027aaffd62",
          "message": "Merge pull request #293 from LoveDaisy/feat/test-time-and-scope-discipline\n\nfeat(ci/doc): 测试时间预算的 owner —— 实测拓扑、分片重装箱、分层契约",
          "timestamp": "2026-09-02T09:04:32+08:00",
          "tree_id": "cfe6beb5c25bbb4014f67bf142635e635a347f4e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc5de377009120dd26063703a16f31027aaffd62"
        },
        "date": 1788311807756,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 87.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "32515f9970bfd614540c36dece2c55fe71eee1a6",
          "message": "Merge pull request #294 from LoveDaisy/chore/annotation-doc-and-diagnostics-gaps\n\nchore(doc/cli): 补 zenith_nadir schema 文档；renderer 超限诊断指向真正的上限",
          "timestamp": "2026-09-02T13:14:20+08:00",
          "tree_id": "377cc3a2fbdf13228f8c678cf3391e0e9631022c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/32515f9970bfd614540c36dece2c55fe71eee1a6"
        },
        "date": 1788326825273,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 96.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "18624004d00f891498779bf12248f36046859a41",
          "message": "Merge pull request #295 from LoveDaisy/feat/fast-e2e-dominant-test\n\ntest(e2e): smoke 按 config 拆成独立 pytest item —— 收集粒度对齐调度粒度，零覆盖损失",
          "timestamp": "2026-09-02T14:42:18+08:00",
          "tree_id": "c633f9361f8df5a4557dd160ab557794d6e71044",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/18624004d00f891498779bf12248f36046859a41"
        },
        "date": 1788332018088,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1ac3ab63b1d72eb8034782f0b8b8f09e100b4636",
          "message": "Merge pull request #296 from LoveDaisy/task/save-open-visual-consistency-red\n\nfix(gui): .lmc 与 composite 纹理改存纯辐亮度，渐晕由显示端统一补上",
          "timestamp": "2026-09-02T18:00:52+08:00",
          "tree_id": "b7bbed427787b088d61b2fa2e14ac122e1150a1d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1ac3ab63b1d72eb8034782f0b8b8f09e100b4636"
        },
        "date": 1788344057173,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "08aa283330101f2ed499cab2ab657abe4bcbb2b2",
          "message": "Merge pull request #297 from LoveDaisy/feat/lens-projection-semantics\n\nfeat(core): 收口 ProjectExitToPixel 遗留的三条 core↔GUI 分歧（参考图只重拍一次）",
          "timestamp": "2026-09-02T23:11:02+08:00",
          "tree_id": "db3c8f71967ab4435d5b915e7c9047561643c424",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/08aa283330101f2ed499cab2ab657abe4bcbb2b2"
        },
        "date": 1788362670038,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dfe34bc13b012f367d913460738f8dc02a50faa0",
          "message": "Merge pull request #298 from LoveDaisy/chore/gui-unit-heartbeat-wallclock-margin\n\ntest(gui-unit): 心跳用例改 wait-until，墙钟余量 250ms → 秒级",
          "timestamp": "2026-09-03T01:12:40+08:00",
          "tree_id": "1b9799920e428da55ee87172d76c0d6689c879f4",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfe34bc13b012f367d913460738f8dc02a50faa0"
        },
        "date": 1788369964890,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "82310d302119a07fff51338e811ffd189ee0aac1",
          "message": "Merge pull request #299 from LoveDaisy/feat/relative-ev-anchor\n\nfeat(core): 把 relative 曝光锚点做对 —— 锚到固定全天缓冲，CLI 与 GUI 消费同一个数",
          "timestamp": "2026-09-03T15:07:51+08:00",
          "tree_id": "582e122487ca1d78c26a6dae0bb7dc213f067af6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/82310d302119a07fff51338e811ffd189ee0aac1"
        },
        "date": 1788420081446,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 69.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c3c5042e4f17c60ecba3cbf6ff38267594826fa",
          "message": "Merge pull request #300 from LoveDaisy/fix/scene-cnt-publish-ordering\n\nfix(server): 记账先于发布，消除批次静默丢失的竞态窗口",
          "timestamp": "2026-09-03T19:19:00+08:00",
          "tree_id": "a4a02453a42967340bf873f3b689410b89d032b9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6c3c5042e4f17c60ecba3cbf6ff38267594826fa"
        },
        "date": 1788435090471,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "365ba58f71d4354ec1035328c12ffd3427c6492e",
          "message": "Merge pull request #301 from LoveDaisy/feat/crystal-slider-domain-and-format\n\nfeat(gui): 柱晶高度域扩到 1e-4，滑杆显示格式由映射闭式定下界并在编译期判错",
          "timestamp": "2026-09-04T15:31:05+08:00",
          "tree_id": "c649e8f7c8eff961a4f64eb3b5956f027157ad32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/365ba58f71d4354ec1035328c12ffd3427c6492e"
        },
        "date": 1788507861552,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ddc94c02faa23de01ef59b90d25f106843699744",
          "message": "Merge pull request #302 from LoveDaisy/fix/preview-source-gather-half-texel\n\nfix(gui): 预览 shader 从 dual-EA 源纹理取样时多出的半个纹素",
          "timestamp": "2026-09-04T18:44:53+08:00",
          "tree_id": "5bbccc2a028e253d4dade882c65deb804b594261",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ddc94c02faa23de01ef59b90d25f106843699744"
        },
        "date": 1788519445690,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7381ca162338ea5105412cc97719e7c7d7eeb1cb",
          "message": "Merge pull request #303 from LoveDaisy/feat/alloc-churn-and-rng-bounds\n\nperf(core): all_data 缓冲区复用 + GetUniform 下标边界收敛到单一 owner",
          "timestamp": "2026-09-04T20:16:18+08:00",
          "tree_id": "f4f4cdc4118328fb3d085e6dd57776fb9c3f53b7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7381ca162338ea5105412cc97719e7c7d7eeb1cb"
        },
        "date": 1788524966671,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1271f2699d6339b658ee96d3cf0502055c2b034a",
          "message": "Merge pull request #304 from LoveDaisy/fix/gui-preview-single-render-path\n\nfix(gui): 预览与 Screenshot 导出收敛到单一离屏 FBO 渲染路径",
          "timestamp": "2026-09-04T20:53:45+08:00",
          "tree_id": "1ea3fd858b1979513692e7dd728217e33f1635d7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1271f2699d6339b658ee96d3cf0502055c2b034a"
        },
        "date": 1788527143942,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61328fbedb7c6acfaf163c040563f24285d52ff2",
          "message": "Merge pull request #305 from LoveDaisy/feat/sky-reference-points\n\nfeat(gui,core): 天空参考点标记六点泛化 + Look At 视角预设，共享 core 单源方向表",
          "timestamp": "2026-09-05T02:15:06+08:00",
          "tree_id": "66f5811bd94711b21e96cefdfa28e8d5b7dcf841",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61328fbedb7c6acfaf163c040563f24285d52ff2"
        },
        "date": 1788546560712,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9327bb7a34e0095c493f6408acc9067e2512fd61",
          "message": "Merge pull request #307 from LoveDaisy/fix/gui-test-harness-gates\n\ntest(gui): 闭合 gui_test 现场三处守卫可信度缺口——恒红闸 / 缺失的互比闸 / 注释纪律一致性",
          "timestamp": "2026-09-05T14:10:18+08:00",
          "tree_id": "2ca052b6512de4cebc819bec0838eb5723d8a739",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9327bb7a34e0095c493f6408acc9067e2512fd61"
        },
        "date": 1788589265575,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 69.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "1f3a9c864ad290679f4f638c62e9bbf6be03a855",
          "message": "Merge pull request #307 from LoveDaisy/fix/gui-test-harness-gates\n\ntest(gui): 闭合 gui_test 现场三处守卫可信度缺口——恒红闸 / 缺失的互比闸 / 注释纪律一致性",
          "timestamp": "2026-09-05T14:15:53+08:00",
          "tree_id": "2ca052b6512de4cebc819bec0838eb5723d8a739",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1f3a9c864ad290679f4f638c62e9bbf6be03a855"
        },
        "date": 1788590065273,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 72.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 98.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fa044a6142525e887ef3e31b89027a953f00343b",
          "message": "Merge pull request #308 from LoveDaisy/fix/clamp-uniform-zero-n\n\nfix(core): ClampUniformToIndex 的 n==0 前提由 assert 改为 FatalAbort 硬守卫",
          "timestamp": "2026-09-05T14:40:02+08:00",
          "tree_id": "b9b2c7baef1b585fa751319ac718c1a01132d770",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa044a6142525e887ef3e31b89027a953f00343b"
        },
        "date": 1788591153129,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "92c4036ebd9dc10187397db16e7a435ba69af19b",
          "message": "Merge pull request #309 from LoveDaisy/chore/native-arch-measurement-hygiene\n\nfeat(bench): 让「本地构建不是出货二进制」这件事在取数字的地方可见（ISA 出处）",
          "timestamp": "2026-09-05T14:54:12+08:00",
          "tree_id": "d6e704e339bd4d14891a0549aafe20bd3799c40e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/92c4036ebd9dc10187397db16e7a435ba69af19b"
        },
        "date": 1788592059203,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 89.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5aea061e953c620bfe6709bf6f2ce87103402f05",
          "message": "Merge pull request #310 from LoveDaisy/fix/cli-label-viewport-clamp\n\nfix(cli): overlay label 视口 clamp 收敛为 GUI/CLI 共享的单一实现",
          "timestamp": "2026-09-05T15:09:23+08:00",
          "tree_id": "f073e0e74a8ffe5cbb3e2967c3330a884cb4279c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5aea061e953c620bfe6709bf6f2ce87103402f05"
        },
        "date": 1788592961402,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f1822e52c6d59ab2c203aa7ef388cb2b50f4e2df",
          "message": "Merge pull request #311 from LoveDaisy/feat/gpu-backend-user-preference\n\nfeat(gui): \"Use GPU\" 可存为个人默认（覆盖文件新增 app 根键），工厂默认仍是 CPU",
          "timestamp": "2026-09-05T15:44:47+08:00",
          "tree_id": "b61f1340307f90b27cb42c9f0d3c8a930fc8a63c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1822e52c6d59ab2c203aa7ef388cb2b50f4e2df"
        },
        "date": 1788594960911,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5015321af574f4bc5efa1787fa3feefc87a835b",
          "message": "Merge pull request #312 from LoveDaisy/fix/benchmark-steady-window-outlier\n\nfix(bench): active_short 不得拿 IDLE 检测延迟当分母（14-29x 野值的根因）",
          "timestamp": "2026-09-05T15:59:52+08:00",
          "tree_id": "e502d8ac59d36d9acdffade736d7bf32b619fa08",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5015321af574f4bc5efa1787fa3feefc87a835b"
        },
        "date": 1788595947438,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fdf903868b533c085c845322ee50722158e3a089",
          "message": "Merge pull request #313 from LoveDaisy/scrum/sentinel-liveness-audit\n\ntest(sentinel): retire the one regression sentinel that outlived its mechanism",
          "timestamp": "2026-09-06T00:54:43+08:00",
          "tree_id": "58522174f803a4d43425b4cd8e96c32dbab08dee",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fdf903868b533c085c845322ee50722158e3a089"
        },
        "date": 1788628076965,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "aa3ec3edc826ec5ba4b7cb59fb1e102222309889",
          "message": "Merge pull request #314 from LoveDaisy/scrum/worker-and-batch-granularity\n\nperf(cpu): cap the automatic worker count at 10, expose --workers, and fix a hit-loop buffer overflow",
          "timestamp": "2026-09-06T10:48:35+08:00",
          "tree_id": "0f52437edfb35a55cb432bf05c2ba64797af8a5f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/aa3ec3edc826ec5ba4b7cb59fb1e102222309889"
        },
        "date": 1788663724514,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5cb81a9565ff0e1394a957d97ad2078d0d8f9310",
          "message": "Merge pull request #315 from LoveDaisy/scrum/changelog-backfill-and-release-notes\n\ndocs(release): 回填 v4.1.4 起 31 个版本的 CHANGELOG，并把它接进发版链路",
          "timestamp": "2026-09-06T16:24:48+08:00",
          "tree_id": "079d5a016ac9ac51339233ba0779369c19e64745",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5cb81a9565ff0e1394a957d97ad2078d0d8f9310"
        },
        "date": 1788683899356,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 76.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c04ad137d42e4a3ed51cd137b75d2d97092b7864",
          "message": "Merge pull request #316 from LoveDaisy/fix/msvc-string-literal-limit\n\nfix(gui,ci): 拆开超 MSVC 上限的 shader 字面量 + 立静态门禁 + CI 触发去重",
          "timestamp": "2026-09-06T23:22:44+08:00",
          "tree_id": "6cd1e5d94251d724205eb0763696031080e86c9d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c04ad137d42e4a3ed51cd137b75d2d97092b7864"
        },
        "date": 1788708810363,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "349656ac94b6b2fd27ecbf4ed35812bea7646a2a",
          "message": "Merge pull request #317 from LoveDaisy/ci/windows-release-image-unify\n\nci: build Windows on the image we actually release from",
          "timestamp": "2026-09-07T01:04:20+08:00",
          "tree_id": "d7e945127aac502e4a16adeca4de66c2fca33ff5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/349656ac94b6b2fd27ecbf4ed35812bea7646a2a"
        },
        "date": 1788714903512,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "59208e341a7a63e1f22366ec0fbc47211fd93950",
          "message": "Merge pull request #318 from LoveDaisy/test/e2e-cost-and-oracle-audit\n\ntest(e2e): 按「每个测试为自己的开销举证」审计套件成本，恢复预算余量",
          "timestamp": "2026-09-07T04:18:31+08:00",
          "tree_id": "a180e65114ec4dbebe9febd562ebcb9d7dcb6dcd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/59208e341a7a63e1f22366ec0fbc47211fd93950"
        },
        "date": 1788726580118,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 88.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e707b15d31a6676c8f4147a0b0cfe62dfc452995",
          "message": "Merge pull request #319 from LoveDaisy/fix/gui-entry-delete-vs-open-editor\n\nfix(gui): keep the edit modal bound to its entry across a delete",
          "timestamp": "2026-09-08T11:14:03+08:00",
          "tree_id": "493abe69703c95a59432e4a6f6623947ade2f407",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e707b15d31a6676c8f4147a0b0cfe62dfc452995"
        },
        "date": 1788837912651,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 106.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "da9e5533acc8c01c61877b6613ddf40bdce9a8b4",
          "message": "Merge pull request #320 from LoveDaisy/fix/cuda-zero-ray-batch-poisons-backend\n\nfix(cuda): stop a zero-ray layer from poisoning the CUDA backend",
          "timestamp": "2026-09-08T17:13:37+08:00",
          "tree_id": "915905aba2ebcbe1abe8eebe327b96458d905c22",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/da9e5533acc8c01c61877b6613ddf40bdce9a8b4"
        },
        "date": 1788859469241,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 73.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d66985dfc19d7e0a2ad278bfb29e8adec87f3adc",
          "message": "Merge pull request #322 from LoveDaisy/test/random-source-exact-assertion-audit\n\ntest: audit random sources behind exact assertions, and refill the lost closed-form fuzz",
          "timestamp": "2026-09-08T19:05:43+08:00",
          "tree_id": "559a5d866b5f56b5751d9d1be56655c2412ea1a2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d66985dfc19d7e0a2ad278bfb29e8adec87f3adc"
        },
        "date": 1788866204883,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 76.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5d738bc07fade93832384583fd5655dda496ae4",
          "message": "Merge pull request #321 from LoveDaisy/ci/organization-and-windows-testing\n\nci(windows): route MSVC compilation through sccache",
          "timestamp": "2026-09-08T20:34:51+08:00",
          "tree_id": "8cbe8c8f0a06164c7e6f448e8e1b9b8f2b5aeff3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5d738bc07fade93832384583fd5655dda496ae4"
        },
        "date": 1788871604718,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 81.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5dab8ac5279acc30e74281d28ec9852f7091260f",
          "message": "Merge pull request #323 from LoveDaisy/feat/annotation-label-line-independence\n\nfeat(config): give the three grid families a line switch of their own",
          "timestamp": "2026-09-08T21:51:50+08:00",
          "tree_id": "17e102e9476ace17c070d7edc5d9863355c01883",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5dab8ac5279acc30e74281d28ec9852f7091260f"
        },
        "date": 1788876191569,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e06d6f8ee003f53159f4265e0a60478c3912298f",
          "message": "Merge pull request #324 from LoveDaisy/perf/cli-render-poll-floor\n\nperf(cli): poll completion before sleeping, so a render is not floored at 1s",
          "timestamp": "2026-09-08T22:52:18+08:00",
          "tree_id": "0e0c25fa9ce2a27fc0554ad989a39b2c64786891",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e06d6f8ee003f53159f4265e0a60478c3912298f"
        },
        "date": 1788879788663,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dfb3f72cde1303813ed87b0403da0bf2d5264b86",
          "message": "Merge pull request #325 from LoveDaisy/fix/user-run-vs-backpressure-gate\n\nfix(gui): exempt a user-initiated Run from the commit backpressure gate",
          "timestamp": "2026-09-08T23:07:48+08:00",
          "tree_id": "3e55872359ee24f0c6224f4e627e5dd6964a1d7d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfb3f72cde1303813ed87b0403da0bf2d5264b86"
        },
        "date": 1788880732587,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61afdda60a81f3800fe9d39d0f1790efbed2eb82",
          "message": "Merge pull request #326 from LoveDaisy/ci/cuda-test-tu-compile-coverage\n\nci: compile the CUDA test TUs (close the CUDA×BUILD_TEST empty intersection)",
          "timestamp": "2026-09-09T09:01:05+08:00",
          "tree_id": "d8015a4a11ce1b395d085be4867bade1cd75f056",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61afdda60a81f3800fe9d39d0f1790efbed2eb82"
        },
        "date": 1788916301959,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5d04ed2b510cf8da1331d69d8a0ec9d064e8688a",
          "message": "Merge pull request #328 from LoveDaisy/feat/gui-import-capability-boundary\n\nfeat(gui): warn on intentionally unsupported capabilities when importing core/CLI configs",
          "timestamp": "2026-09-09T11:28:47+08:00",
          "tree_id": "5fb7bd048fbeeb0720b37f54dc406c24b4f696f7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5d04ed2b510cf8da1331d69d8a0ec9d064e8688a"
        },
        "date": 1788925163996,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fe4c0778ad67356b7107c49b9f2d1758751cbab0",
          "message": "Merge pull request #329 from LoveDaisy/fix/raypath-load-path-syntax-gate\n\nfix(gui): reject malformed raypath summand rows on the .lmc load path",
          "timestamp": "2026-09-09T11:50:12+08:00",
          "tree_id": "85ae4abe8324f97a506977a0b6b5e2f1ee1a6194",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fe4c0778ad67356b7107c49b9f2d1758751cbab0"
        },
        "date": 1788926447909,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 67.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 81.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "61199dc71bb8afa824f182ecb361c1765212e2ba",
          "message": "Merge pull request #330 from LoveDaisy/build/cpm-cache-shared-default\n\nbuild(cpm): default the dependency-source cache to a machine-level directory",
          "timestamp": "2026-09-09T12:28:59+08:00",
          "tree_id": "401ce6afc69f30c5242ddcb0e9b1c84c265275ad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/61199dc71bb8afa824f182ecb361c1765212e2ba"
        },
        "date": 1788928790150,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "52b769f8831d3826607f24929ab7130e9dc62d1e",
          "message": "Merge pull request #331 from LoveDaisy/refactor/field-set-sentinel-proxy\n\nrefactor(config): guard RenderConfig's field set by member count, not sizeof",
          "timestamp": "2026-09-09T13:13:40+08:00",
          "tree_id": "ea00f972f5073c2d5a34b75fbe200d19d12f07b6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/52b769f8831d3826607f24929ab7130e9dc62d1e"
        },
        "date": 1788931515598,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "96d644a21248a0968f4866679a3de372c1610833",
          "message": "Merge pull request #334 from LoveDaisy/feat/bg-image-color-picker\n\nfeat(gui): sample Sky Color off the background photo with an eyedropper",
          "timestamp": "2026-09-10T01:16:27+08:00",
          "tree_id": "dacea434bc3720c060ce99ea1e1f7083321478c0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/96d644a21248a0968f4866679a3de372c1610833"
        },
        "date": 1788974906928,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "258d9d34fde41b6c3c29d2818e91a4a20c13f2af",
          "message": "Merge pull request #335 from LoveDaisy/ci/drop-unused-vendor-apt-source\n\nci: stop depending on a vendor apt source nothing here installs from",
          "timestamp": "2026-09-10T02:19:42+08:00",
          "tree_id": "0ec8ef1453363cf2a4e9bac9403be312d6a3d461",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/258d9d34fde41b6c3c29d2818e91a4a20c13f2af"
        },
        "date": 1788978637042,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "712eb886076cecd28b4dedc683f8255351558cb5",
          "message": "Merge pull request #333 from LoveDaisy/ci/cache-budget\n\nci(cache): budget the actions/cache quota — fix three prefix-shadowed keys, add ccache to the critical-path leg",
          "timestamp": "2026-09-10T02:35:58+08:00",
          "tree_id": "c53075d893ac829084799f406bdd8c280a195292",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/712eb886076cecd28b4dedc683f8255351558cb5"
        },
        "date": 1788979619952,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 74.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 92.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "491b117b9a07cdf85de8099529f7811e686abf1e",
          "message": "Merge pull request #336 from LoveDaisy/feat/miller-index-and-wedge-presets\n\nfix(gui,core): give the Miller-index wedge conversion one owner, and correct the presets it was never checked against",
          "timestamp": "2026-09-10T04:26:14+08:00",
          "tree_id": "4e290141061128a452482994544759c4c4475a08",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/491b117b9a07cdf85de8099529f7811e686abf1e"
        },
        "date": 1788986315645,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a4c526050e74287deacd4473631926deabf13a9",
          "message": "Merge pull request #337 from LoveDaisy/feat/print-mode-subtractive-ink\n\nfeat(render,gui): add a print tone that lays ink on paper instead of adding light to sky",
          "timestamp": "2026-09-10T09:06:10+08:00",
          "tree_id": "7d1fa0a3596ea0279c8d776fa7d4f0baaa8feab6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a4c526050e74287deacd4473631926deabf13a9"
        },
        "date": 1789003115561,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 76.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ae46f7c283910d4fbb4e7a0f2949aef02e5f87df",
          "message": "Merge pull request #338 from LoveDaisy/feat/gui-display-rendering-regroup\n\nfix(gui): regroup the Display Rendering rows and pair the ground swatch with the mode",
          "timestamp": "2026-09-10T14:08:18+08:00",
          "tree_id": "eeaec146eaa0226974bb83e95e4e1b36f34970a9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ae46f7c283910d4fbb4e7a0f2949aef02e5f87df"
        },
        "date": 1789021108360,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 93.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3ad57411dc16f516b6785967efaba5266c88e7b8",
          "message": "Merge pull request #339 from LoveDaisy/feat/test-capi-lib\n\ntest: liblumice_testapi, a test-only export surface beside the product C API",
          "timestamp": "2026-09-10T16:59:47+08:00",
          "tree_id": "91bfa648700adc1c02137e2bab552ca4271f3417",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3ad57411dc16f516b6785967efaba5266c88e7b8"
        },
        "date": 1789031377316,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "724fa7cff5ff0cc832f33d45b08e1bf3d4536f40",
          "message": "Merge pull request #342 from LoveDaisy/feat/annotation-lines-shader-anchors-api\n\ngui: auxiliary lines track the camera every frame again; anchors-only annotation API (v4.28)",
          "timestamp": "2026-09-10T17:18:09+08:00",
          "tree_id": "7f9537581734b6e612b1d500271e44df92102186",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/724fa7cff5ff0cc832f33d45b08e1bf3d4536f40"
        },
        "date": 1789032513828,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d57f132dc2f546ad001a215fb28d6c917bddabf4",
          "message": "Merge pull request #340 from LoveDaisy/docs/working-discipline-hardening\n\ndocs+hooks: harden two working-discipline rules into criteria and a commit gate",
          "timestamp": "2026-09-10T18:06:39+08:00",
          "tree_id": "1e03b6ab5fd5688bd565295581883fc4c9690d85",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d57f132dc2f546ad001a215fb28d6c917bddabf4"
        },
        "date": 1789035357696,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "37b141504a40cc0be937f0d5bf071fef24759171",
          "message": "Merge pull request #341 from LoveDaisy/test/defaults-panel-refs-reshoot\n\ntest(gui): pin the wedge add row in every preset scene, and re-shoot the two that were not",
          "timestamp": "2026-09-10T18:49:54+08:00",
          "tree_id": "42fe951d730ad0cd31b0d30b5c6fc14fa0ec2dc1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37b141504a40cc0be937f0d5bf071fef24759171"
        },
        "date": 1789038034008,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f860acc46471dee85057842a8611895cea64b88d",
          "message": "Merge pull request #343 from LoveDaisy/feat/gui-print-mode-label-ink\n\nfix(gui): draw overlay label text as ink under the print tone",
          "timestamp": "2026-09-10T20:57:53+08:00",
          "tree_id": "f3daf403c69c5adec882772328d1a77ec96a9215",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f860acc46471dee85057842a8611895cea64b88d"
        },
        "date": 1789045698466,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 70.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dc76b64939e2b7e7bb319dee15905da1c73ee7fa",
          "message": "Merge pull request #344 from LoveDaisy/feat/image-comparison-metric-by-layer\n\ntest: give each image comparison a ruler that matches its layer (pixel ruler, lines-only parity, block-mean PSNR)",
          "timestamp": "2026-09-11T01:37:19+08:00",
          "tree_id": "41b16f85c1532f746a24d73f00d8388af7f2060a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dc76b64939e2b7e7bb319dee15905da1c73ee7fa"
        },
        "date": 1789062633837,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 86.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "38aff9c6a97f3fdcca9801ffeb6e1dcecf4be998",
          "message": "Merge pull request #345 from LoveDaisy/chore/release-4.5.1\n\nrelease: cut 4.5.1, and make the release a per-version backfill chore",
          "timestamp": "2026-09-11T08:06:39+08:00",
          "tree_id": "c20bb077289e78ac31076d801efb94c50cab02ad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/38aff9c6a97f3fdcca9801ffeb6e1dcecf4be998"
        },
        "date": 1789085821398,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "142e29e615d7a573006eabf610b33948e5584c98",
          "message": "Merge pull request #346 from LoveDaisy/feat/hardware-perf-distribution\n\nbuild/release: ship ISA- and GPU-matched binaries behind CPUID launchers (x86-64-v4 Linux, x86-64-v3 clang-cl Windows, sm_120 fatbin)",
          "timestamp": "2026-09-11T20:51:32+08:00",
          "tree_id": "f05bd3485353b55d626d7d9fe93091774693bb97",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/142e29e615d7a573006eabf610b33948e5584c98"
        },
        "date": 1789131616296,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 94.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4fdfe61cc4b1326f60a924a669133f7c6311023d",
          "message": "Merge pull request #347 from LoveDaisy/feat/raypath-analysis-panel\n\nfeat: raypath analysis panel — dedicated non-rendering pass, ranked by chain",
          "timestamp": "2026-09-12T16:27:12+08:00",
          "tree_id": "071aa48f973504cccddec5a5f2199596e22adfc9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4fdfe61cc4b1326f60a924a669133f7c6311023d"
        },
        "date": 1789202423219,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "70eb8f5fad44336cf2b57da4d314a9aced4c8224",
          "message": "Merge pull request #349 from LoveDaisy/feat/raypath-analysis-followups\n\nRaypath analysis follow-ups: fixed-seed reproducibility, session-kind rebuild predicate, joiner glyphs, debt sweep",
          "timestamp": "2026-09-12T23:46:53+08:00",
          "tree_id": "1570515bb91fdd1a4610a3ac00dc77a6ae1f7cca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/70eb8f5fad44336cf2b57da4d314a9aced4c8224"
        },
        "date": 1789228788442,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c106292be4122b60092a9666131dde843f639b15",
          "message": "Merge pull request #348 from LoveDaisy/feat/crystal-ray-allocation\n\nfeat(core): adaptive ray allocation across crystal entries (scene.ray_allocation)",
          "timestamp": "2026-09-13T04:19:28+08:00",
          "tree_id": "82448d3731df653e760db492e5a8794a13dc6f0c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c106292be4122b60092a9666131dde843f639b15"
        },
        "date": 1789245358053,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 92.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "88e0fbf6864b1d95ba7e19c4d6660fb8c15c1f4c",
          "message": "Merge pull request #350 from LoveDaisy/chore/install-manual-refresh-and-review-minors\n\nchore: refresh the install manual, land the metric-by-layer review minors, report wrong-size anchor planes once",
          "timestamp": "2026-09-13T04:52:15+08:00",
          "tree_id": "94bf3da366cd365b6946ce53b5309cd6df95c36e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/88e0fbf6864b1d95ba7e19c4d6660fb8c15c1f4c"
        },
        "date": 1789247025108,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d5c230764f43779ffb32bc75e454ec07f2159a03",
          "message": "Merge pull request #351 from LoveDaisy/fix/exposure-mode-combo-fixed-separation\n\ntest(gui): prove exposure-mode separation with an intensity probe, not a seed-dependent gap",
          "timestamp": "2026-09-13T05:13:06+08:00",
          "tree_id": "8da3f1ccd7d22ef20d6fff6dff9a5c615f8e013d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d5c230764f43779ffb32bc75e454ec07f2159a03"
        },
        "date": 1789248088818,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7a68880e398137ef91895edbb6d4ff0c997923e4",
          "message": "Merge pull request #352 from LoveDaisy/chore/regen-refs-deterministic-single-shot\n\nchore(regen-refs): shoot deterministic groups once, share runs across groups, refuse stale-base reshoots",
          "timestamp": "2026-09-13T05:31:09+08:00",
          "tree_id": "5e9ceaec07873c2b9154175e6d957784999b8820",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7a68880e398137ef91895edbb6d4ff0c997923e4"
        },
        "date": 1789249344475,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "365409776ad9761a5ebf3402cf9cde48f573e9d8",
          "message": "Merge pull request #353 from LoveDaisy/fix/render-consumer-label-flake-root-cause\n\nfix(test): root-cause the RenderConsumerLabel flake — an uninitialized SunParam azimuth",
          "timestamp": "2026-09-13T05:47:24+08:00",
          "tree_id": "2a620f7705b88d7f56d29a1cf923e190e358fd9e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/365409776ad9761a5ebf3402cf9cde48f573e9d8"
        },
        "date": 1789250203207,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 92.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "35235a01c6962917a905abe105b458ee0ba444ab",
          "message": "Merge pull request #354 from LoveDaisy/feat/ray-num-slider-100b-log-scale\n\nfeat(gui): Rays(M) slider spans 0.1..100 000 M on a kLog track, one domain for both rows",
          "timestamp": "2026-09-13T06:17:09+08:00",
          "tree_id": "8c31257305277f134a3473abfb64eca3bdbdbc3a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35235a01c6962917a905abe105b458ee0ba444ab"
        },
        "date": 1789252147861,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f0add0b4a676a2e20ab27c782e9e9b5a182aac5b",
          "message": "Merge pull request #355 from LoveDaisy/feat/cli-lens-and-grid-contract\n\nfeat(lens): the CLI/GUI lens contract — short-edge fov, defaults, focal length import, annotations at intensity 0",
          "timestamp": "2026-09-13T07:31:34+08:00",
          "tree_id": "14ed237826bf2e9217fc60dc8c0bd508aee5669a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f0add0b4a676a2e20ab27c782e9e9b5a182aac5b"
        },
        "date": 1789256498533,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 76.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 87.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "51fa59e29850545fd09b7f6041faaed4a4bea4cd",
          "message": "Merge pull request #356 from LoveDaisy/feat/cuda-hostgen-black-and-energy-accounting\n\nfix(cuda): host root-gen fallback renders again; landed weight reduced per warp so the energy ledger matches legacy",
          "timestamp": "2026-09-13T08:09:28+08:00",
          "tree_id": "5310d81311d369114cdde3eb97a22ec84b3b27c2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51fa59e29850545fd09b7f6041faaed4a4bea4cd"
        },
        "date": 1789258765905,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.5,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "22166140295c68e58e6375028394d4587a351c11",
          "message": "Merge pull request #357 from LoveDaisy/feat/view-center-angular-dist-grid\n\nfeat(annotation): view_dist — circles of constant angular distance from the optical axis, config → core → C API → GUI",
          "timestamp": "2026-09-13T12:02:01+08:00",
          "tree_id": "6875aee1957344381ca66a902bb0dc2ba20f8f02",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/22166140295c68e58e6375028394d4587a351c11"
        },
        "date": 1789273864943,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.2,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "682bf9aadbd77a61c2d7697ccd4b353bb70a06be",
          "message": "Merge pull request #358 from LoveDaisy/fix/equidistant-focal-length-factor-two\n\nfix(config): equidistant lens f→fov conversion was half the documented value",
          "timestamp": "2026-09-13T12:42:20+08:00",
          "tree_id": "ad66fbc16fa5d96511d2e7667a3348c6c69b86d5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/682bf9aadbd77a61c2d7697ccd4b353bb70a06be"
        },
        "date": 1789275331756,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 87.8,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b10d1bbc134afb64423fa842b7740de5a59934f2",
          "message": "Merge pull request #359 from LoveDaisy/feat/analysis-panel-polish\n\nfeat(gui): raypath analysis panel polish — first-picture gate, draw layer, geometry, thousands grouping, Export CSV",
          "timestamp": "2026-09-13T14:55:21+08:00",
          "tree_id": "ffbfcccfae868fd3b9504d3d98acd998d41a1933",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b10d1bbc134afb64423fa842b7740de5a59934f2"
        },
        "date": 1789283180008,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 89.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "90d23cafd492e0f65663df5a681c439b0fa09f35",
          "message": "Merge pull request #360 from LoveDaisy/feat/analysis-standing-cpu-pool\n\nfeat(server): standing CPU analysis pool on the GPU route, woken by session kind",
          "timestamp": "2026-09-13T15:15:21+08:00",
          "tree_id": "0e81a696990d2128a156faf657717d6b2838d130",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/90d23cafd492e0f65663df5a681c439b0fa09f35"
        },
        "date": 1789284495186,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 63.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.7,
            "unit": "%"
          }
        ]
      }
    ]
  }
}