window.BENCHMARK_DATA = {
  "lastUpdate": 1775706599375,
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
          "id": "8d7f4e2fda303a7cc2a7af7f40b3b510ee6c1c9f",
          "message": "Merge pull request #48 from LoveDaisy/feat/refactor_logger\n\nrefactor(core): remove LUMICE_InitLogger, decouple logger from server lifecycle",
          "timestamp": "2026-04-06T00:41:43+08:00",
          "tree_id": "b7ee785ca4ba9c280432d5769395837a5f16a3df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d7f4e2fda303a7cc2a7af7f40b3b510ee6c1c9f"
        },
        "date": 1775407654505,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 355272.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608484.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432815.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 379474.8,
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
          "id": "77619b84a60bbf93eab1a8e448b94826e5daccf2",
          "message": "Merge pull request #49 from LoveDaisy/feat/json_api\n\nfeat(c-api): add JSON→LUMICE_Config parsing API",
          "timestamp": "2026-04-06T01:18:01+08:00",
          "tree_id": "56a3de0263a49d8b728c2c05d297ae5384bfdfa0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/77619b84a60bbf93eab1a8e448b94826e5daccf2"
        },
        "date": 1775409676415,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 466346.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627772,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432757.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 378059.2,
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
          "id": "5548e8acc740ce89034441662cae0cb19c816a29",
          "message": "Merge pull request #50 from LoveDaisy/feat/improve_filter\n\nfix(filter): DirectionFilter correctness fix and filter subsystem improvements",
          "timestamp": "2026-04-06T11:58:35+08:00",
          "tree_id": "8e6aad4814519815f04ee9170aa02e23067e2d32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5548e8acc740ce89034441662cae0cb19c816a29"
        },
        "date": 1775448148720,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 334981.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627594.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432758,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 381243.3,
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
          "id": "10f0e13e274b0d2f0da17b5a5be993d033af1711",
          "message": "Merge pull request #51 from LoveDaisy/exp/sim_efficiency\n\nfix(core): Jacobian-corrected spherical sampling with proper folding",
          "timestamp": "2026-04-06T18:51:38+08:00",
          "tree_id": "80c1ba94edd691159926145f3891693afd7cb725",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/10f0e13e274b0d2f0da17b5a5be993d033af1711"
        },
        "date": 1775472910932,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 472584.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647837.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414767.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 380223.9,
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
          "id": "9fd5b962e42b671f21ccf7e61bf19d462902708f",
          "message": "Merge pull request #52 from LoveDaisy/feat/axis_dist\n\nfeat: add zigzag and laplacian axis distribution types",
          "timestamp": "2026-04-06T23:31:58+08:00",
          "tree_id": "866611c6d3effac07c263c82c88d0f61c5625a57",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9fd5b962e42b671f21ccf7e61bf19d462902708f"
        },
        "date": 1775489752350,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 477185.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627618.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432870.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 369380.9,
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
          "id": "7afaf1ee9852b2a5b2fca35706f11e8cb5cd621d",
          "message": "Merge pull request #53 from LoveDaisy/feat/crystal_param\n\nfeat: replace Miller index with wedge angle for pyramid crystals",
          "timestamp": "2026-04-07T11:45:57+08:00",
          "tree_id": "8676799829f22d664374bd7c645615e57c324396",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7afaf1ee9852b2a5b2fca35706f11e8cb5cd621d"
        },
        "date": 1775533770390,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 443682.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627577.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423599.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 440322.4,
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
          "id": "ba2780cbd4f7d833fbb6ea8fbe1d10c882645c90",
          "message": "Merge pull request #54 from LoveDaisy/feat/gui_layout\n\nfeat(gui): restructure layout with collapsible panels and overlay",
          "timestamp": "2026-04-08T08:36:58+08:00",
          "tree_id": "2c5b3cc1492cebe092283ae3adfa7bbd060dad26",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ba2780cbd4f7d833fbb6ea8fbe1d10c882645c90"
        },
        "date": 1775608831819,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 445583.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627338,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423585.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 376414.2,
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
          "id": "d4eae674d89a686b081a2aa818bb653152bfdbf3",
          "message": "Merge pull request #55 from LoveDaisy/feat/gui_overlay\n\nfeat(gui): configurable overlay colors, alpha, and edge labels",
          "timestamp": "2026-04-08T08:57:53+08:00",
          "tree_id": "caed3f29268640848e2a7ef86a5e23b0407ecf68",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d4eae674d89a686b081a2aa818bb653152bfdbf3"
        },
        "date": 1775610076478,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 373203.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647955.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 404994,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 376028.2,
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
          "id": "37e473a268869d7c47912cd6488c48d17d3cc090",
          "message": "Merge pull request #56 from LoveDaisy/feat/gui_misc\n\nGUI misc polish: layout, controls, overlay labels",
          "timestamp": "2026-04-08T14:56:40+08:00",
          "tree_id": "d6c4dcba4e00d8eeaef1a15e4179e88fd1095ec0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37e473a268869d7c47912cd6488c48d17d3cc090"
        },
        "date": 1775631617466,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 444498.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627548.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432719.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 334972.7,
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
          "id": "c5250b8e42fbe86232200d4da64fde36d8fde191",
          "message": "Merge pull request #57 from LoveDaisy/feat/doc\n\ndocs: fix factual errors, restructure research docs, update dev guide",
          "timestamp": "2026-04-08T15:56:42+08:00",
          "tree_id": "82368eda0f53caea2bce8c9048bc818ce2aa084c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c5250b8e42fbe86232200d4da64fde36d8fde191"
        },
        "date": 1775635203249,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 458172.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627504.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389086.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 432040.5,
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
          "id": "51a7c57890eafc5c0773329a1d52f0fb1a64d6d3",
          "message": "Merge pull request #58 from LoveDaisy/feat/toolbar_ux\n\nGUI panel UX improvements and toolbar polish",
          "timestamp": "2026-04-08T22:47:18+08:00",
          "tree_id": "6b94a3971124f54864e7b09e50f91de9ff6ebdd0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51a7c57890eafc5c0773329a1d52f0fb1a64d6d3"
        },
        "date": 1775659847279,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 413388.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627523.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396904.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 384607.2,
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
          "id": "95e7699b2bba2318532067b0d3d12d9e4ce99068",
          "message": "Merge pull request #59 from LoveDaisy/fix/misc\n\nfix: misc bug fixes for core simulator and GUI",
          "timestamp": "2026-04-09T10:57:52+08:00",
          "tree_id": "1a782734a4d4771b5ff2efb2ce8ac6c94a79fc21",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/95e7699b2bba2318532067b0d3d12d9e4ce99068"
        },
        "date": 1775703690652,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 435584.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647872.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423502.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337540.5,
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
          "id": "f10e150db7b5ddd631fc6474c0d7895ea65d11d9",
          "message": "Merge pull request #60 from LoveDaisy/fix/dual_fisheye_overlap\n\nfeat(config): make dual fisheye overlap configurable",
          "timestamp": "2026-04-09T11:46:17+08:00",
          "tree_id": "f08f3c6849c30a704334f0097d28dbc2d4a51390",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f10e150db7b5ddd631fc6474c0d7895ea65d11d9"
        },
        "date": 1775706598453,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 465651.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627644.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396956.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 381571.4,
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
          "id": "8d7f4e2fda303a7cc2a7af7f40b3b510ee6c1c9f",
          "message": "Merge pull request #48 from LoveDaisy/feat/refactor_logger\n\nrefactor(core): remove LUMICE_InitLogger, decouple logger from server lifecycle",
          "timestamp": "2026-04-06T00:41:43+08:00",
          "tree_id": "b7ee785ca4ba9c280432d5769395837a5f16a3df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d7f4e2fda303a7cc2a7af7f40b3b510ee6c1c9f"
        },
        "date": 1775407656052,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1001373.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1958863.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1242734.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1046579.5,
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
          "id": "77619b84a60bbf93eab1a8e448b94826e5daccf2",
          "message": "Merge pull request #49 from LoveDaisy/feat/json_api\n\nfeat(c-api): add JSON→LUMICE_Config parsing API",
          "timestamp": "2026-04-06T01:18:01+08:00",
          "tree_id": "56a3de0263a49d8b728c2c05d297ae5384bfdfa0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/77619b84a60bbf93eab1a8e448b94826e5daccf2"
        },
        "date": 1775409677907,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1243592.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1996766.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1241382.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1059621.8,
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
          "id": "5548e8acc740ce89034441662cae0cb19c816a29",
          "message": "Merge pull request #50 from LoveDaisy/feat/improve_filter\n\nfix(filter): DirectionFilter correctness fix and filter subsystem improvements",
          "timestamp": "2026-04-06T11:58:35+08:00",
          "tree_id": "8e6aad4814519815f04ee9170aa02e23067e2d32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5548e8acc740ce89034441662cae0cb19c816a29"
        },
        "date": 1775448150830,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 980849.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 2038118,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1240899.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1039358.9,
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
          "id": "10f0e13e274b0d2f0da17b5a5be993d033af1711",
          "message": "Merge pull request #51 from LoveDaisy/exp/sim_efficiency\n\nfix(core): Jacobian-corrected spherical sampling with proper folding",
          "timestamp": "2026-04-06T18:51:38+08:00",
          "tree_id": "80c1ba94edd691159926145f3891693afd7cb725",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/10f0e13e274b0d2f0da17b5a5be993d033af1711"
        },
        "date": 1775472913017,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1171631,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1959496.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1241733.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1031323.9,
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
          "id": "9fd5b962e42b671f21ccf7e61bf19d462902708f",
          "message": "Merge pull request #52 from LoveDaisy/feat/axis_dist\n\nfeat: add zigzag and laplacian axis distribution types",
          "timestamp": "2026-04-06T23:31:58+08:00",
          "tree_id": "866611c6d3effac07c263c82c88d0f61c5625a57",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9fd5b962e42b671f21ccf7e61bf19d462902708f"
        },
        "date": 1775489755120,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1111081.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1918438.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1242272.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1047150.4,
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
          "id": "7afaf1ee9852b2a5b2fca35706f11e8cb5cd621d",
          "message": "Merge pull request #53 from LoveDaisy/feat/crystal_param\n\nfeat: replace Miller index with wedge angle for pyramid crystals",
          "timestamp": "2026-04-07T11:45:57+08:00",
          "tree_id": "8676799829f22d664374bd7c645615e57c324396",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7afaf1ee9852b2a5b2fca35706f11e8cb5cd621d"
        },
        "date": 1775533773029,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1152281.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 2036475,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1225063.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 990665.9,
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
          "id": "ba2780cbd4f7d833fbb6ea8fbe1d10c882645c90",
          "message": "Merge pull request #54 from LoveDaisy/feat/gui_layout\n\nfeat(gui): restructure layout with collapsible panels and overlay",
          "timestamp": "2026-04-08T08:36:58+08:00",
          "tree_id": "2c5b3cc1492cebe092283ae3adfa7bbd060dad26",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ba2780cbd4f7d833fbb6ea8fbe1d10c882645c90"
        },
        "date": 1775608834031,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1158088.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1961656.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1241682.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1053924.5,
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
          "id": "d4eae674d89a686b081a2aa818bb653152bfdbf3",
          "message": "Merge pull request #55 from LoveDaisy/feat/gui_overlay\n\nfeat(gui): configurable overlay colors, alpha, and edge labels",
          "timestamp": "2026-04-08T08:57:53+08:00",
          "tree_id": "caed3f29268640848e2a7ef86a5e23b0407ecf68",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d4eae674d89a686b081a2aa818bb653152bfdbf3"
        },
        "date": 1775610078125,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1066133.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1960339.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1195118.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1047996.6,
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
          "id": "37e473a268869d7c47912cd6488c48d17d3cc090",
          "message": "Merge pull request #56 from LoveDaisy/feat/gui_misc\n\nGUI misc polish: layout, controls, overlay labels",
          "timestamp": "2026-04-08T14:56:40+08:00",
          "tree_id": "d6c4dcba4e00d8eeaef1a15e4179e88fd1095ec0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37e473a268869d7c47912cd6488c48d17d3cc090"
        },
        "date": 1775631619146,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1184594.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1960618.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1255996.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 995862,
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
          "id": "c5250b8e42fbe86232200d4da64fde36d8fde191",
          "message": "Merge pull request #57 from LoveDaisy/feat/doc\n\ndocs: fix factual errors, restructure research docs, update dev guide",
          "timestamp": "2026-04-08T15:56:42+08:00",
          "tree_id": "82368eda0f53caea2bce8c9048bc818ce2aa084c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c5250b8e42fbe86232200d4da64fde36d8fde191"
        },
        "date": 1775635205636,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1253943.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 2036509.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1209912.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1008036.8,
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
          "id": "51a7c57890eafc5c0773329a1d52f0fb1a64d6d3",
          "message": "Merge pull request #58 from LoveDaisy/feat/toolbar_ux\n\nGUI panel UX improvements and toolbar polish",
          "timestamp": "2026-04-08T22:47:18+08:00",
          "tree_id": "6b94a3971124f54864e7b09e50f91de9ff6ebdd0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51a7c57890eafc5c0773329a1d52f0fb1a64d6d3"
        },
        "date": 1775659850479,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1154213.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1958190.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1195172.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1057583.1,
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
          "id": "95e7699b2bba2318532067b0d3d12d9e4ce99068",
          "message": "Merge pull request #59 from LoveDaisy/fix/misc\n\nfix: misc bug fixes for core simulator and GUI",
          "timestamp": "2026-04-09T10:57:52+08:00",
          "tree_id": "1a782734a4d4771b5ff2efb2ce8ac6c94a79fc21",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/95e7699b2bba2318532067b0d3d12d9e4ce99068"
        },
        "date": 1775703692626,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 989062.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1921597.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1242887.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 992440.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
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
          "id": "8d7f4e2fda303a7cc2a7af7f40b3b510ee6c1c9f",
          "message": "Merge pull request #48 from LoveDaisy/feat/refactor_logger\n\nrefactor(core): remove LUMICE_InitLogger, decouple logger from server lifecycle",
          "timestamp": "2026-04-06T00:41:43+08:00",
          "tree_id": "b7ee785ca4ba9c280432d5769395837a5f16a3df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d7f4e2fda303a7cc2a7af7f40b3b510ee6c1c9f"
        },
        "date": 1775407657086,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 94,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 80.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 68.9,
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
          "id": "77619b84a60bbf93eab1a8e448b94826e5daccf2",
          "message": "Merge pull request #49 from LoveDaisy/feat/json_api\n\nfeat(c-api): add JSON→LUMICE_Config parsing API",
          "timestamp": "2026-04-06T01:18:01+08:00",
          "tree_id": "56a3de0263a49d8b728c2c05d297ae5384bfdfa0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/77619b84a60bbf93eab1a8e448b94826e5daccf2"
        },
        "date": 1775409678893,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 79.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 70.1,
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
          "id": "5548e8acc740ce89034441662cae0cb19c816a29",
          "message": "Merge pull request #50 from LoveDaisy/feat/improve_filter\n\nfix(filter): DirectionFilter correctness fix and filter subsystem improvements",
          "timestamp": "2026-04-06T11:58:35+08:00",
          "tree_id": "8e6aad4814519815f04ee9170aa02e23067e2d32",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5548e8acc740ce89034441662cae0cb19c816a29"
        },
        "date": 1775448152122,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 97.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 81.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 68.2,
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
          "id": "10f0e13e274b0d2f0da17b5a5be993d033af1711",
          "message": "Merge pull request #51 from LoveDaisy/exp/sim_efficiency\n\nfix(core): Jacobian-corrected spherical sampling with proper folding",
          "timestamp": "2026-04-06T18:51:38+08:00",
          "tree_id": "80c1ba94edd691159926145f3891693afd7cb725",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/10f0e13e274b0d2f0da17b5a5be993d033af1711"
        },
        "date": 1775472914360,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 67.8,
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
          "id": "9fd5b962e42b671f21ccf7e61bf19d462902708f",
          "message": "Merge pull request #52 from LoveDaisy/feat/axis_dist\n\nfeat: add zigzag and laplacian axis distribution types",
          "timestamp": "2026-04-06T23:31:58+08:00",
          "tree_id": "866611c6d3effac07c263c82c88d0f61c5625a57",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9fd5b962e42b671f21ccf7e61bf19d462902708f"
        },
        "date": 1775489756969,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 76.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 70.9,
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
          "id": "7afaf1ee9852b2a5b2fca35706f11e8cb5cd621d",
          "message": "Merge pull request #53 from LoveDaisy/feat/crystal_param\n\nfeat: replace Miller index with wedge angle for pyramid crystals",
          "timestamp": "2026-04-07T11:45:57+08:00",
          "tree_id": "8676799829f22d664374bd7c645615e57c324396",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7afaf1ee9852b2a5b2fca35706f11e8cb5cd621d"
        },
        "date": 1775533774547,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 81.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 56.2,
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
          "id": "ba2780cbd4f7d833fbb6ea8fbe1d10c882645c90",
          "message": "Merge pull request #54 from LoveDaisy/feat/gui_layout\n\nfeat(gui): restructure layout with collapsible panels and overlay",
          "timestamp": "2026-04-08T08:36:58+08:00",
          "tree_id": "2c5b3cc1492cebe092283ae3adfa7bbd060dad26",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ba2780cbd4f7d833fbb6ea8fbe1d10c882645c90"
        },
        "date": 1775608835311,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 78.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 70,
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
          "id": "d4eae674d89a686b081a2aa818bb653152bfdbf3",
          "message": "Merge pull request #55 from LoveDaisy/feat/gui_overlay\n\nfeat(gui): configurable overlay colors, alpha, and edge labels",
          "timestamp": "2026-04-08T08:57:53+08:00",
          "tree_id": "caed3f29268640848e2a7ef86a5e23b0407ecf68",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d4eae674d89a686b081a2aa818bb653152bfdbf3"
        },
        "date": 1775610079146,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 95.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 69.7,
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
          "id": "37e473a268869d7c47912cd6488c48d17d3cc090",
          "message": "Merge pull request #56 from LoveDaisy/feat/gui_misc\n\nGUI misc polish: layout, controls, overlay labels",
          "timestamp": "2026-04-08T14:56:40+08:00",
          "tree_id": "d6c4dcba4e00d8eeaef1a15e4179e88fd1095ec0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/37e473a268869d7c47912cd6488c48d17d3cc090"
        },
        "date": 1775631620193,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 78.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 74.3,
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
          "id": "c5250b8e42fbe86232200d4da64fde36d8fde191",
          "message": "Merge pull request #57 from LoveDaisy/feat/doc\n\ndocs: fix factual errors, restructure research docs, update dev guide",
          "timestamp": "2026-04-08T15:56:42+08:00",
          "tree_id": "82368eda0f53caea2bce8c9048bc818ce2aa084c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c5250b8e42fbe86232200d4da64fde36d8fde191"
        },
        "date": 1775635206948,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 91.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 81.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 77.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 58.3,
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
          "id": "51a7c57890eafc5c0773329a1d52f0fb1a64d6d3",
          "message": "Merge pull request #58 from LoveDaisy/feat/toolbar_ux\n\nGUI panel UX improvements and toolbar polish",
          "timestamp": "2026-04-08T22:47:18+08:00",
          "tree_id": "6b94a3971124f54864e7b09e50f91de9ff6ebdd0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51a7c57890eafc5c0773329a1d52f0fb1a64d6d3"
        },
        "date": 1775659852564,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 93.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 78,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 75.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 68.7,
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
          "id": "95e7699b2bba2318532067b0d3d12d9e4ce99068",
          "message": "Merge pull request #59 from LoveDaisy/fix/misc\n\nfix: misc bug fixes for core simulator and GUI",
          "timestamp": "2026-04-09T10:57:52+08:00",
          "tree_id": "1a782734a4d4771b5ff2efb2ce8ac6c94a79fc21",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/95e7699b2bba2318532067b0d3d12d9e4ce99068"
        },
        "date": 1775703693820,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 73.5,
            "unit": "%"
          }
        ]
      }
    ]
  }
}