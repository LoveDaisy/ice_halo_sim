window.BENCHMARK_DATA = {
  "lastUpdate": 1782266703954,
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1f75c3ab8ea1beb25a0388b63e801e66ef298674",
          "message": "Merge pull request #61 from LoveDaisy/feat/legacy_axis_dist\n\nfeat(core): add legacy Gaussian distribution type",
          "timestamp": "2026-04-09T14:17:22+08:00",
          "tree_id": "3100700029c59b35185531fcee8ac4da29d06826",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1f75c3ab8ea1beb25a0388b63e801e66ef298674"
        },
        "date": 1775715686397,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 380327,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647953.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 404935.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 385570.7,
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
          "id": "8a3f0bc88f188c376f4274fef6055b64c6500561",
          "message": "Merge pull request #62 from LoveDaisy/fix/ray_alloc\n\nFix ray allocation starvation and C API config limits",
          "timestamp": "2026-04-09T19:30:20+08:00",
          "tree_id": "18ba2e31c99ba6aebf53957871f90c6f0d305fed",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8a3f0bc88f188c376f4274fef6055b64c6500561"
        },
        "date": 1775734439364,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 434632.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627726.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396804.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 443741.2,
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
          "id": "7e14e2262bc6720b96deb8ac82df7ff23de37c90",
          "message": "Merge pull request #63 from LoveDaisy/feat/test_coverage\n\ntest: close zero-coverage gaps in optics, c_api, sim_data + fix doc errors",
          "timestamp": "2026-04-10T16:30:44+08:00",
          "tree_id": "be592e1fb54dc3f40eb93d1f98d64bcd28e8a26a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7e14e2262bc6720b96deb8ac82df7ff23de37c90"
        },
        "date": 1775810083958,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 364216,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627357.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423569.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 351915.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a80668db54c90087c70fafdb840db88857e9ab07",
          "message": "build(deps): bump actions/download-artifact from 7 to 8 (#64)\n\nBumps [actions/download-artifact](https://github.com/actions/download-artifact) from 7 to 8.\n- [Release notes](https://github.com/actions/download-artifact/releases)\n- [Commits](https://github.com/actions/download-artifact/compare/v7...v8)\n\n---\nupdated-dependencies:\n- dependency-name: actions/download-artifact\n  dependency-version: '8'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>",
          "timestamp": "2026-04-12T20:14:47+08:00",
          "tree_id": "6d94fbf535ea3956bed86891a3a2035db6454167",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a80668db54c90087c70fafdb840db88857e9ab07"
        },
        "date": 1775996350893,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 453178.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 648050.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 523701.3,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 350966.5,
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
          "id": "024de6454b199b86311707d9ca7f02a104a83f75",
          "message": "Merge pull request #65 from LoveDaisy/feat/test_improve\n\ntest: audit phase 2 — 62 new tests + SampleSphCapPoint rotation fix",
          "timestamp": "2026-04-12T20:23:36+08:00",
          "tree_id": "e79853846edab103efd7e96dad743f4ea39848fe",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/024de6454b199b86311707d9ca7f02a104a83f75"
        },
        "date": 1775996836757,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 461903.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647383.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 497491.3,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 370338.8,
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
          "id": "12a30449eb7e798bdfa1f470ae2212f6aa518ead",
          "message": "Merge pull request #66 from LoveDaisy/feat/test_org\n\nReorganize test files: split GUI tests and separate integration tests",
          "timestamp": "2026-04-12T21:53:12+08:00",
          "tree_id": "d528ebc7c90a4396f461148ebfd75a5ede8a3e47",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/12a30449eb7e798bdfa1f470ae2212f6aa518ead"
        },
        "date": 1776002247799,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 403988.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647957,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432783.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 380465.2,
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
          "id": "25c74ca230184e57181e36fa4fab523e0623dc7e",
          "message": "Merge pull request #67 from LoveDaisy/feat/ux_tweak\n\nfeat(gui): four UX interaction improvements",
          "timestamp": "2026-04-13T10:27:55+08:00",
          "tree_id": "2f82826d00b92f5318caad5d530ba688bc406bf3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/25c74ca230184e57181e36fa4fab523e0623dc7e"
        },
        "date": 1776047571329,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 374465.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627658.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 510107.4,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 340927.3,
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
          "id": "f97dd7cb850710474f5c4580a3e2c57b75814c36",
          "message": "Merge pull request #68 from LoveDaisy/feat/gui_refactor\n\nrefactor(gui): card-based layer/entry layout with copy model",
          "timestamp": "2026-04-14T10:39:15+08:00",
          "tree_id": "ca973ccab53593acaf7fd1bf3fd594e6fd860385",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f97dd7cb850710474f5c4580a3e2c57b75814c36"
        },
        "date": 1776134637162,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 409165.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 648088.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423617.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 380121.4,
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
          "id": "ff9fddc949d519efbacf1e341aadf4dd6327f50a",
          "message": "Merge pull request #69 from LoveDaisy/feat/gui_refactor_debt\n\nrefactor(gui): resolve GUI-refactor tech debt + GUI test regressions",
          "timestamp": "2026-04-14T16:31:10+08:00",
          "tree_id": "dc3e20ffaf9b7855bf0448f1dd6042e830aa6ac4",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ff9fddc949d519efbacf1e341aadf4dd6327f50a"
        },
        "date": 1776155753604,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 395705.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 626879.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 404963.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 339706.6,
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
          "id": "104f13eb374ca98ef511f2f3f98d2fa05069feec",
          "message": "Merge pull request #71 from LoveDaisy/feat/gui_polish\n\nGUI polish: edit modal unification, card layout v2, v7 UX fixes",
          "timestamp": "2026-04-19T22:31:08+08:00",
          "tree_id": "5c6045501eb4a202b506b3d96688144625083561",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/104f13eb374ca98ef511f2f3f98d2fa05069feec"
        },
        "date": 1776609312769,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 446620.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627578.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432812.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 380049.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6607e4adbb792a86297bb42d03be771efb7495ee",
          "message": "build(deps): bump softprops/action-gh-release from 2 to 3 (#70)\n\nBumps [softprops/action-gh-release](https://github.com/softprops/action-gh-release) from 2 to 3.\n- [Release notes](https://github.com/softprops/action-gh-release/releases)\n- [Changelog](https://github.com/softprops/action-gh-release/blob/master/CHANGELOG.md)\n- [Commits](https://github.com/softprops/action-gh-release/compare/v2...v3)\n\n---\nupdated-dependencies:\n- dependency-name: softprops/action-gh-release\n  dependency-version: '3'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>",
          "timestamp": "2026-04-20T10:51:03+08:00",
          "tree_id": "c8a08fa588e47013e82196eab0275bf7deddf597",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6607e4adbb792a86297bb42d03be771efb7495ee"
        },
        "date": 1776653768633,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 366269.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627489.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423571.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345649.8,
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
          "id": "14383572230a4e449571244b98e6c988a07527b2",
          "message": "Merge pull request #72 from LoveDaisy/feat/gui_polish\n\nfeat(gui): GUI polish — overlay labels, 导出管线重构 & modal 立即模式",
          "timestamp": "2026-04-21T19:02:28+08:00",
          "tree_id": "46011cd81e57f4a345bfd3f6a1eb819208300a17",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/14383572230a4e449571244b98e6c988a07527b2"
        },
        "date": 1776769639871,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 391176,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647818.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 510195,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 374409.7,
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
          "id": "6ea03e48619368357ee574c62b5c31388d6af037",
          "message": "Merge pull request #73 from LoveDaisy/feat/gui_polish\n\nfeat(gui-polish-v12): 7 子任务 — 滑杆/录屏/Immediate/弹窗布局/face number",
          "timestamp": "2026-04-22T18:41:08+08:00",
          "tree_id": "214de347db4c3436af50e2b7a55d1d805945c514",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6ea03e48619368357ee574c62b5c31388d6af037"
        },
        "date": 1776854753846,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 369383.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627668.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423608.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337395.9,
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
          "id": "6690144f4a571a7b62131587d5620b8f258c777b",
          "message": "Merge pull request #74 from LoveDaisy/feat/gui_polish\n\nGUI polish: Immediate modal, filter guards, face-number overlay refine",
          "timestamp": "2026-04-23T18:26:03+08:00",
          "tree_id": "dcac66ac084932f8e7a41d73b925f2e1f4abfe50",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6690144f4a571a7b62131587d5620b8f258c777b"
        },
        "date": 1776940233555,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 310472.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647696.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423570.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 376940,
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
          "id": "e63069a2c407830a197ccf0afe383644959d4b8e",
          "message": "Merge pull request #75 from LoveDaisy/feat/gui_polish\n\nGUI polish v15: orthographic lens, modal H/V layout, detachable Immediate modal",
          "timestamp": "2026-04-24T09:59:54+08:00",
          "tree_id": "568063651932670da5f27b983647c867d4a87abc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e63069a2c407830a197ccf0afe383644959d4b8e"
        },
        "date": 1776996262483,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 353808.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647748.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396937,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 383489.3,
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
          "id": "08a2db71b84673557445aaceeeaed31fd09aa279",
          "message": "Merge pull request #76 from LoveDaisy/feat/coordinate_system\n\nCoordinate system overhaul: HaloRay v1 convention + modal preview unification",
          "timestamp": "2026-04-26T13:56:57+08:00",
          "tree_id": "d4bf864cd81160a275162336577ca3c88d1d3bd7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/08a2db71b84673557445aaceeeaed31fd09aa279"
        },
        "date": 1777183269869,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 477299.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 648008.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 511908.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 447124.6,
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
          "id": "cd819bf654497d43842d70955ab1150f677de6a5",
          "message": "Merge pull request #77 from LoveDaisy/dev/misc\n\nchore: misc cleanup and version bump to 4.2.2",
          "timestamp": "2026-04-26T23:44:03+08:00",
          "tree_id": "b57679d8ec11db4aec618fac5ac9a2708914a4ef",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cd819bf654497d43842d70955ab1150f677de6a5"
        },
        "date": 1777218489850,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 391058.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647918.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 509861.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 338784.2,
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
          "id": "c88a590f41460f51142553918e19cd6da61b6604",
          "message": "Merge pull request #78 from LoveDaisy/feat/gui_polish\n\nfeat(gui): polish overlay labels and orthographic lens behavior",
          "timestamp": "2026-04-27T14:49:09+08:00",
          "tree_id": "49dcf8c39bac9be1da8ffe00b0684430d03fbb09",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c88a590f41460f51142553918e19cd6da61b6604"
        },
        "date": 1777272876058,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 354266.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627438.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 442297.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 374540.2,
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
          "id": "6878c2d8eca3eb1fcb40efdb7b81fc896a559534",
          "message": "Merge pull request #79 from LoveDaisy/feat/gui_polish\n\nfeat(gui): aspect ratio clamp warning + overlay label sampler refactor",
          "timestamp": "2026-04-28T17:49:24+08:00",
          "tree_id": "5c03cdff293a4b10bd73c61099429e2cbbf1b25e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6878c2d8eca3eb1fcb40efdb7b81fc896a559534"
        },
        "date": 1777370030742,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 315145.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647765.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423565.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 431313.9,
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
          "id": "ac272dc408c8e4a5df4ffcbaac9688ce08f0d1ac",
          "message": "Merge pull request #80 from LoveDaisy/feat/doc\n\ndocs: restructure README + add user manual + GUI guide rewrite",
          "timestamp": "2026-04-29T02:54:01+08:00",
          "tree_id": "0358f20d18af1bdf98c12138ee71013d445dc914",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ac272dc408c8e4a5df4ffcbaac9688ce08f0d1ac"
        },
        "date": 1777402704262,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 461829.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 648118.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396937.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 374492.3,
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
          "id": "26e54569e59d8e18420bf4a3a58d92d268b91b5a",
          "message": "Merge pull request #81 from LoveDaisy/fix/modal_zorder\n\nfix(gui): show combo popups above detached Edit Entry modal",
          "timestamp": "2026-04-29T15:54:43+08:00",
          "tree_id": "6e2ad77922fffeadb6e2da7e47e4fa9f66c34300",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/26e54569e59d8e18420bf4a3a58d92d268b91b5a"
        },
        "date": 1777449559582,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 429091.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608289.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432789.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 376391.8,
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
          "id": "620c3717415d7c99e46bf9718a151c931cf669e0",
          "message": "Merge pull request #82 from LoveDaisy/feat/overlay_line_label\n\nfeat(gui): split overlay Line / Label toggles",
          "timestamp": "2026-04-29T17:52:06+08:00",
          "tree_id": "7007374414365f508bf36f92cc6f9453d14e7746",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/620c3717415d7c99e46bf9718a151c931cf669e0"
        },
        "date": 1777456598741,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 405998.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 648053.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432786.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 335747.9,
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
          "id": "847970a791d9ccde2a8d62467763289fc672771b",
          "message": "Merge pull request #83 from LoveDaisy/feat/globe_view\n\nfeat(gui): Globe lens with trackball, overlay labels, and view reset",
          "timestamp": "2026-04-30T23:23:42+08:00",
          "tree_id": "4c980cbd42b23b410fd148ee16eada28b2feccdb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/847970a791d9ccde2a8d62467763289fc672771b"
        },
        "date": 1777562894454,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 393891.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647939.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423616.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 437854.9,
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
          "id": "5c889e1e1f526d6bda40f029153821ff8a06fcf6",
          "message": "Merge pull request #84 from LoveDaisy/feat/ux_crystal_card\n\nfeat(gui): highlight active crystal card while edit modal is open",
          "timestamp": "2026-05-01T00:42:19+08:00",
          "tree_id": "1fe7d7881a3ce16e3cdfd44fdececc046f8f1afd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c889e1e1f526d6bda40f029153821ff8a06fcf6"
        },
        "date": 1777567615361,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 377136.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647978.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 432904.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 436872.5,
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
          "id": "efafa43354b35f824c02394d62dbc328464441ba",
          "message": "Merge pull request #85 from LoveDaisy/feat/more_filters\n\nfeat: per-type filter subpanels (Raypath / EntryExit / Direction / Crystal) with .lmc v2 serialization",
          "timestamp": "2026-05-06T11:32:12+08:00",
          "tree_id": "c65cfd36e780f84271a95de1b162d2f55d96cbbf",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/efafa43354b35f824c02394d62dbc328464441ba"
        },
        "date": 1778038617702,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 372463.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627695.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414916.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373973.8,
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
          "id": "f04e1ad447d4f4ccdeef564955f7043422e706e6",
          "message": "Merge pull request #86 from LoveDaisy/feat/globe_view\n\nfeat: Globe↔other lens combo transform, visibility radio buttons, View panel regroup",
          "timestamp": "2026-05-06T12:27:33+08:00",
          "tree_id": "3291df96efef44c52eb9b8d5a8f0c2f7a1722641",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f04e1ad447d4f4ccdeef564955f7043422e706e6"
        },
        "date": 1778041920768,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 413357.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 648217.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 511820.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 419095,
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
          "id": "629b323b216916714ed81f72579bb132517b2088",
          "message": "Merge pull request #87 from LoveDaisy/feat/gui_polish\n\nfeat(gui-polish-v18): visibility row, drop Direction filter, EE align Raypath",
          "timestamp": "2026-05-07T12:18:21+08:00",
          "tree_id": "cba0b9455d12d07982d550091a56b3c28e98d010",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/629b323b216916714ed81f72579bb132517b2088"
        },
        "date": 1778127812269,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 412867.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627782.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423677.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 372198.6,
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
          "id": "702fec607e094345d83386ea8011b67d9ccc5718",
          "message": "Merge pull request #88 from LoveDaisy/feat/raypath_symmetry\n\nRaypath P/B/D symmetry redesign + fold/roll fix + e2e regression",
          "timestamp": "2026-05-09T12:44:27+08:00",
          "tree_id": "73a61c0e6a037827a94616b45044f837a5cd9b4f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/702fec607e094345d83386ea8011b67d9ccc5718"
        },
        "date": 1778302155642,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 370079.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647965.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423438.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373494.2,
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
          "id": "940557be35656483ccb5c98a4312d48e7a6b8359",
          "message": "Merge pull request #89 from LoveDaisy/feat/auto_ev\n\nfeat: Adaptive Brightness (P99-anchored auto-EV) for GUI",
          "timestamp": "2026-05-13T09:57:49+08:00",
          "tree_id": "503baee7a3d600f6b16282f53e78474e03be1b8a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/940557be35656483ccb5c98a4312d48e7a6b8359"
        },
        "date": 1778637758799,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 468532.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627297.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396850.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 376747.2,
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
          "id": "94e4edf61c07a64640c38d90c1b2dc3fda974653",
          "message": "Merge pull request #90 from LoveDaisy/dev/refactor_c_api\n\nRefactor: enforce GUI/core API boundary via 4 new C API entries",
          "timestamp": "2026-05-14T20:00:37+08:00",
          "tree_id": "99603c8703022d6dc31ab9fa8440052434a0ba84",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/94e4edf61c07a64640c38d90c1b2dc3fda974653"
        },
        "date": 1778760315278,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 479123.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 647323,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 509626.6,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 356642.5,
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
          "id": "8d97704a2cc32149478a5e9fade9bfca321483d6",
          "message": "Merge pull request #91 from LoveDaisy/feat/update_test_ref\n\nUpdate GUI test references with mean-ref pipeline",
          "timestamp": "2026-05-14T20:18:50+08:00",
          "tree_id": "a2a8ea7ee23efaa4c5c619cec9b8b223fcaf5f5b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d97704a2cc32149478a5e9fade9bfca321483d6"
        },
        "date": 1778761397952,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 353545.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 626694.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396843.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 441319.6,
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
          "id": "280bf0be7058816e808ba6a78cc98a7a5452eb6b",
          "message": "Merge pull request #92 from LoveDaisy/fix/raypath-1-3\n\nfix(core): repair raypath=[1,3] TIR edge leak via fid_in_src parameter",
          "timestamp": "2026-05-18T08:53:02+08:00",
          "tree_id": "8587ee20db4172496e5a7a3bd75b60244f2d8958",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/280bf0be7058816e808ba6a78cc98a7a5452eb6b"
        },
        "date": 1779065889770,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 432606.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608202.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 496619.6,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373682.2,
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
          "id": "fa8e6cfb354f00e5922cc8d458b330670ff7fb29",
          "message": "Merge pull request #93 from LoveDaisy/feat/query-filter-uplift-v2\n\nfeat(core): uplift query filter to RenderConsumer, fix OFF-mode normalization",
          "timestamp": "2026-05-18T10:04:37+08:00",
          "tree_id": "25b4ddbbd0c316a6ae1f3b02b2309815af457d9c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa8e6cfb354f00e5922cc8d458b330670ff7fb29"
        },
        "date": 1779070170439,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 341941.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608250.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414637.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 442914.5,
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
          "id": "b30b552674d681adb8b6123bcbea5954b3340e2f",
          "message": "Merge pull request #94 from LoveDaisy/feat/partition-additivity-test-redesign\n\nRedesign test_partition_buffer_additivity with discriminating metric",
          "timestamp": "2026-05-18T14:17:11+08:00",
          "tree_id": "8c193a6d69f9da7200a7ef970dd3842024d97fb2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b30b552674d681adb8b6123bcbea5954b3340e2f"
        },
        "date": 1779085342228,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 452127.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627232.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414660.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337514.6,
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
          "id": "de1ce7ae90f4d4404c0fc55b92f4a267c52da934",
          "message": "Merge pull request #95 from LoveDaisy/feat/mesh_per_face\n\nfeat(core): scrum-193 leftovers batch — per-face topology + polygon-only tracing",
          "timestamp": "2026-05-18T18:12:13+08:00",
          "tree_id": "e40e53678e278c0e378b085931d4091084ba7f98",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de1ce7ae90f4d4404c0fc55b92f4a267c52da934"
        },
        "date": 1779099451680,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 310795.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 572578.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 390398.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 351537.7,
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
          "id": "5908ce3f2446e604b823aeea72293bb87451f589",
          "message": "Merge pull request #96 from LoveDaisy/dev/refactor_rayseg\n\nrefactor: redesign RaySeg state + sentinel (scrum-206)",
          "timestamp": "2026-05-19T09:32:23+08:00",
          "tree_id": "f209b2da889e6a16d9d82f574c0cc0695d8140f3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5908ce3f2446e604b823aeea72293bb87451f589"
        },
        "date": 1779154648747,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 425176.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 626377.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389063.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373521,
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
          "id": "c680dcc6a69433924731db037eb416920fd012a4",
          "message": "Merge pull request #97 from LoveDaisy/feat/editor_window_ux\n\nfeat(gui): edit modal UX polish (raypath buffer + pyramid slider alignment)",
          "timestamp": "2026-05-19T11:22:18+08:00",
          "tree_id": "b896575144a36ab9ccd7b0d844d85b5edf0a7929",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c680dcc6a69433924731db037eb416920fd012a4"
        },
        "date": 1779161217057,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 340039.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608294.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398075.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 366885.2,
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
          "id": "35e35f3275c09d626a2ebadc49c026d8d790ff17",
          "message": "Merge pull request #98 from LoveDaisy/feat/filter_performance\n\nfeat(core): canonical-form FilterSpec + fix per-ray symmetry hot path",
          "timestamp": "2026-05-20T09:41:53+08:00",
          "tree_id": "fb556e78e23a0fafa29a2be152e168d43f33bfa3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35e35f3275c09d626a2ebadc49c026d8d790ff17"
        },
        "date": 1779241575917,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 291791.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 607945,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414714.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 439216.1,
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
          "id": "ea5ab5ed603b0df052058857dc75ee0a822d6df4",
          "message": "Merge pull request #99 from LoveDaisy/feat/ux_polish\n\nfeat(gui): ux polish v2 + dead-code cleanup (panels.cpp::SliderWithPreset)",
          "timestamp": "2026-05-20T11:55:13+08:00",
          "tree_id": "286ea92e04c4c205a715bcc8d8867c7166e0bcdd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ea5ab5ed603b0df052058857dc75ee0a822d6df4"
        },
        "date": 1779249583619,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 376609.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627198.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 381647.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 367250.2,
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
          "id": "2506276291a8c48473a44af1f0e849671d260ff4",
          "message": "Merge pull request #100 from LoveDaisy/fix/capi\n\nfix(c-api): guard sentinel write past max_count + slow regression test",
          "timestamp": "2026-05-20T16:47:27+08:00",
          "tree_id": "64f28a12c3e3144dd31b56a51b4ab9f90d1e4a96",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2506276291a8c48473a44af1f0e849671d260ff4"
        },
        "date": 1779267134308,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 387510.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608438.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396826.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 367165.6,
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
          "id": "dec91f357ae70588ef872acc48f9e3268e0d21aa",
          "message": "Merge pull request #101 from LoveDaisy/fix/capi_shared_lib\n\nfix(gui-shared-lib): expose XyzToSrgbUint8 via C API; restore GUI/core boundary",
          "timestamp": "2026-05-20T18:14:57+08:00",
          "tree_id": "5f0da1e0c5f0e2ce6ff584f3de125f66f308436c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dec91f357ae70588ef872acc48f9e3268e0d21aa"
        },
        "date": 1779272416214,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 337991.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 626705.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414689.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 372986.7,
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
          "id": "135151eed06f1cb0288a1020d0a10c52cc45c485",
          "message": "Merge pull request #102 from LoveDaisy/feat/gui_font\n\nfeat(gui): integrate FontAwesome 6 icon font for 12 GUI buttons",
          "timestamp": "2026-05-21T12:50:01+08:00",
          "tree_id": "949a6505c17485e7d2383f3d8f4f9f6d89603285",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/135151eed06f1cb0288a1020d0a10c52cc45c485"
        },
        "date": 1779339323357,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 451976.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627274,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396822.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 313412.4,
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
          "id": "d158a9394600b1ded192e6dc974603f42b76fb47",
          "message": "Merge pull request #103 from LoveDaisy/feat/gui_card_link\n\nfeat(gui): linked entries via ID-pool model + pick-mode UX",
          "timestamp": "2026-05-23T17:41:59+08:00",
          "tree_id": "dcb404af64a0c4302dbf917a45e0f4888b230422",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d158a9394600b1ded192e6dc974603f42b76fb47"
        },
        "date": 1779529612768,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 417999,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627438.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 423504.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373722.3,
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
          "id": "dd136e1208d0ad1d81384632592cc2c9421597ab",
          "message": "Merge pull request #104 from LoveDaisy/fix/ms_prob_leak\n\nfix(filter): restore filter as simulator emit-gate (Design A) + architecture doc",
          "timestamp": "2026-05-23T17:55:00+08:00",
          "tree_id": "e624cd047ec6fa17909614f4f428aa2ee3d96f8f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dd136e1208d0ad1d81384632592cc2c9421597ab"
        },
        "date": 1779530380911,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 433073.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 527944.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396969,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 381297.9,
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
          "id": "e925a47e0db8e111ca3bd397a8ed21c44f8ed48d",
          "message": "Merge pull request #105 from LoveDaisy/fix/adaptive_brightness_screenshot\n\nAdaptive Brightness: single-mode F1 anchor + screenshot fix + P99.5/T135 defaults",
          "timestamp": "2026-05-24T20:37:24+08:00",
          "tree_id": "a7d961fcc9ac82180aab92bb7165fd154a4c4616",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e925a47e0db8e111ca3bd397a8ed21c44f8ed48d"
        },
        "date": 1779626505798,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 465974.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 573339.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414814.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 334257.2,
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
          "id": "fc3e2fa37ad9f4bce408a9a5817f09eb57aa7c8a",
          "message": "Merge pull request #106 from LoveDaisy/feat/gui_zenith_nadir\n\nfeat(gui): zenith/nadir ring marker overlay",
          "timestamp": "2026-05-25T01:23:44+08:00",
          "tree_id": "3f5880b3f64c960c7f36c1b0c568f0b38f820075",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc3e2fa37ad9f4bce408a9a5817f09eb57aa7c8a"
        },
        "date": 1779643716538,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 443952.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 573187.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389233.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 309777.8,
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
          "id": "22dc93834aba6b5b8c0e55d16027165f830004f8",
          "message": "Merge pull request #107 from LoveDaisy/feat/ci_test\n\nci: add slow e2e test job with shared-lib build",
          "timestamp": "2026-05-25T09:25:31+08:00",
          "tree_id": "8bdc09a9c9a6566dab36fe1e61008d26ef894aab",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/22dc93834aba6b5b8c0e55d16027165f830004f8"
        },
        "date": 1779672705775,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 446061,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 573714.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389214.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 317357.2,
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
          "id": "e6da1a081780ca1412d671fc3108615df715b988",
          "message": "Merge pull request #108 from LoveDaisy/fix/ms_filter_leak\n\nfix(simulator): prevent MS filter-leaked rays from reaching main output",
          "timestamp": "2026-05-25T14:17:56+08:00",
          "tree_id": "f0ddc6dd281209f960a42d9ba62a62b1e4389732",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e6da1a081780ca1412d671fc3108615df715b988"
        },
        "date": 1779690118445,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 422267.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 557129.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 414788,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 430783.5,
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
          "id": "b555bce0cfb41ac2959a09682690abaee59e4dcc",
          "message": "Merge pull request #109 from LoveDaisy/dev/env_var\n\nrefactor: replace LUMICE_SIM_SEED env var with C API sim_seed parameter",
          "timestamp": "2026-05-25T15:12:52+08:00",
          "tree_id": "7b0e4e2ac9d8d9d06a5c565821e5ba98db920649",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b555bce0cfb41ac2959a09682690abaee59e4dcc"
        },
        "date": 1779693461920,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 429853.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 573703.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 498250.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 365606.7,
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
          "id": "57735b3bf85ff7ae78b102ea0d93aca76e34cc73",
          "message": "Merge pull request #110 from LoveDaisy/feat/render_refresh\n\nFix rendering update issues, simulator audit gaps, and raise max hits",
          "timestamp": "2026-05-26T01:05:00+08:00",
          "tree_id": "9b9aab40e5072b0b7c0942bdb8ed63184483a299",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/57735b3bf85ff7ae78b102ea0d93aca76e34cc73"
        },
        "date": 1779728982709,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 413407.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 557146.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 390451,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 307608.1,
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
          "id": "9db583df928264e5fe43a90e364da883ed197704",
          "message": "Merge pull request #111 from LoveDaisy/feat/gui_minor\n\nfeat(gui): card-wide click and front-hemisphere checkbox",
          "timestamp": "2026-05-27T08:18:34+08:00",
          "tree_id": "3010b3aeccd240cd41893b564209d19ca5731165",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9db583df928264e5fe43a90e364da883ed197704"
        },
        "date": 1779841357166,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 418280.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 556749.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 390413.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 396338.1,
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
          "id": "062fff11d9ca5a48e9424fc2424572c415b9460f",
          "message": "Merge pull request #112 from LoveDaisy/fix/misc\n\nchore(gui): unify overlay sentinel and add review follow-up fixes",
          "timestamp": "2026-05-27T08:42:42+08:00",
          "tree_id": "adcb4da74d7fbf0987da6ef3c60a107b7c34e64a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/062fff11d9ca5a48e9424fc2424572c415b9460f"
        },
        "date": 1779842844515,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 434645.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 572713.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 375800.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345730.4,
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
          "id": "eb70214863932c839b9f6102a4ce5961856333bc",
          "message": "Merge pull request #113 from LoveDaisy/chore/gui_lag\n\nfix(gui): eliminate high-resolution GUI lag",
          "timestamp": "2026-05-27T10:08:15+08:00",
          "tree_id": "a7433ce2d4a833d2ea80f18d110d7ea3ce15f183",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/eb70214863932c839b9f6102a4ce5961856333bc"
        },
        "date": 1779847930423,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 415145.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 573676.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 382967,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 570007.6,
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
          "id": "e4a3f6587b8ef382e420de0b5d739f4f5bc0d8c5",
          "message": "Merge pull request #114 from LoveDaisy/chore/major_audit\n\nchore: major audit — architecture docs, tooltips, and cross-ref comments",
          "timestamp": "2026-05-28T08:33:03+08:00",
          "tree_id": "aabe60b8c5d31e9c2a26d6fc057ffc6fbfe12dad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e4a3f6587b8ef382e420de0b5d739f4f5bc0d8c5"
        },
        "date": 1779928706581,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 430239.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 572471.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 462959.4,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 334918.7,
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
          "id": "e3d86034ece73c9d5b32a9981fc5b324bdbfbce6",
          "message": "Merge pull request #115 from LoveDaisy/feat/remove_ev_anchor\n\nRemove F1 anchor lane, use self-P99.5 EV normalization",
          "timestamp": "2026-05-28T15:38:16+08:00",
          "tree_id": "52693d70e631b7a71599cdcad16ec33dd9f6c0d1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e3d86034ece73c9d5b32a9981fc5b324bdbfbce6"
        },
        "date": 1779954195951,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 419067.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 573460,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 367645.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 348636.4,
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
          "id": "dc2918aa9d575eeb2b3007788b1bb8cd62f29193",
          "message": "Merge pull request #116 from LoveDaisy/feat/ev-anchor-p99\n\nfeat(auto-ev): rework adaptive brightness anchor (P99 + global f=8 downsample metric)",
          "timestamp": "2026-05-31T20:02:04+08:00",
          "tree_id": "cd1482b671d823bc75c304687bebc49aaa388c11",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dc2918aa9d575eeb2b3007788b1bb8cd62f29193"
        },
        "date": 1780229215275,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 298149.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 557463.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 336163.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 313435,
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
          "id": "dfad7e8b454dfdd7903349cf553465cdcacf91e9",
          "message": "Merge pull request #117 from LoveDaisy/chore/ev-pipeline-doc-sync\n\ndocs: align EV/filter architecture docs to single-lane (post anchor-lane removal)",
          "timestamp": "2026-06-01T00:01:46+08:00",
          "tree_id": "bcc818e5077e221eaa5599a098d72d4c7e60f479",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfad7e8b454dfdd7903349cf553465cdcacf91e9"
        },
        "date": 1780243625341,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 413247.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 557687.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 368836.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 338917.7,
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
          "id": "d726d4f8d498793da4007ea5f1b139f12c7d5a9d",
          "message": "Merge pull request #118 from LoveDaisy/feat/ee-filter-uplift\n\nfeat: EntryExit filter expressiveness — wildcard, multi-value OR, length bounds",
          "timestamp": "2026-06-01T12:57:04+08:00",
          "tree_id": "2d104ee146202faca56b9fd9ed006a45cf215c55",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d726d4f8d498793da4007ea5f1b139f12c7d5a9d"
        },
        "date": 1780290111134,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 345855.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 573559.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 375810.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337532.5,
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
          "id": "a83345b608ebc03c14a674ba2d9e86a766c27e61",
          "message": "Merge pull request #119 from LoveDaisy/dev/soa_refactor\n\nCPU SoA: ray recorder footprint optimization (RaypathRecorder split + SBO)",
          "timestamp": "2026-06-02T14:44:34+08:00",
          "tree_id": "54bf6808db99db93563899e56f190d727a9b2868",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a83345b608ebc03c14a674ba2d9e86a766c27e61"
        },
        "date": 1780382989880,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 358089.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608531.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 486816.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 417429.8,
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
          "id": "392ee041c7255f883dcb83f6eb4c4958ea8f8d28",
          "message": "Merge pull request #120 from LoveDaisy/dev/perf_opt2\n\nperf(worker-default): default worker count to physical cores",
          "timestamp": "2026-06-03T18:05:54+08:00",
          "tree_id": "0863f2e30f743b4cf8bdb1004f290e0da8e43762",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/392ee041c7255f883dcb83f6eb4c4958ea8f8d28"
        },
        "date": 1780481465771,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 381846,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608568.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398177.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 322210.3,
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
          "id": "390813463050266f30c6406eab1cb4b9a52972bb",
          "message": "Merge pull request #121 from LoveDaisy/feat/metal_backend_prod\n\nfeat: Metal trace backend (production) — pluggable TraceBackend seam + CPU/Metal parity",
          "timestamp": "2026-06-08T17:19:20+08:00",
          "tree_id": "ce38b259f1a458387effaafcbece2fdfa5c2e196",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/390813463050266f30c6406eab1cb4b9a52972bb"
        },
        "date": 1780910675000,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 449724.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608373.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 406348.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 326670.8,
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
          "id": "5c588072df470acedf8b0eb419f2c1b278e77239",
          "message": "Merge pull request #122 from LoveDaisy/feat/metal-gui-default\n\nfeat: enable Metal backend on GUI live-preview (dual-fisheye-EA + C API toggle)",
          "timestamp": "2026-06-09T09:33:36+08:00",
          "tree_id": "d8c695861215eb9ce11c1bf5c993c7a78a99cece",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c588072df470acedf8b0eb419f2c1b278e77239"
        },
        "date": 1780969097144,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 431358.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608468.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 381748.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321505.3,
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
          "id": "ca327679e29ff539e1ab7a1b51177ed6066af02f",
          "message": "Merge pull request #123 from LoveDaisy/feat/metal-exit-seam\n\nfeat(metal): exit-seam buffer egress 出口替代整幅回读 (P1/scrum-258)",
          "timestamp": "2026-06-12T09:45:24+08:00",
          "tree_id": "0af735a07c022e2c17115e13424724ac9d00cca0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ca327679e29ff539e1ab7a1b51177ed6066af02f"
        },
        "date": 1781229029824,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 362398.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 606311.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 406356.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 322923.1,
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
          "id": "c5b5985c34ddd97e7372b393a1d7892279bf136c",
          "message": "Merge pull request #124 from LoveDaisy/feat/metal-rootgen\n\nfeat(metal): device GPU root-gen 默认根供给 + 吞吐确证 (P2/scrum-260+261)",
          "timestamp": "2026-06-12T10:11:44+08:00",
          "tree_id": "7e1566edab98bcbd5054d82dd2e28bbc19b99c7e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c5b5985c34ddd97e7372b393a1d7892279bf136c"
        },
        "date": 1781230749967,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 462626.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608666,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 374648.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 354011,
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
          "id": "207de068d30e8080608200a0b1cf9b54c3518f43",
          "message": "Merge pull request #125 from LoveDaisy/fix/serverimpl-stop-deadlock\n\nfix: ServerImpl::Stop() condition-variable lost-wakeup deadlock + stress regression",
          "timestamp": "2026-06-12T11:53:36+08:00",
          "tree_id": "476d50f4d2d83297176fb1ddb991155d55897e02",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/207de068d30e8080608200a0b1cf9b54c3518f43"
        },
        "date": 1781236759885,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 433813,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 607613.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 381830.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 351313.9,
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
          "id": "00f9d49fc95fe411dd82ab6686e21fc0de5718e2",
          "message": "Merge pull request #126 from LoveDaisy/feat/metal-gen-trace-fusion\n\nfeat(metal): fuse gen+trace into one command buffer (task-264)",
          "timestamp": "2026-06-13T01:45:22+08:00",
          "tree_id": "9aed83822638731d531d0ae74a7dc2df39d40d92",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/00f9d49fc95fe411dd82ab6686e21fc0de5718e2"
        },
        "date": 1781286652457,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 352893.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608569.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398241.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321727.3,
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
          "id": "f007991d19cb9c206a4966a0d19133f8838a6762",
          "message": "Merge pull request #127 from LoveDaisy/docs/gpu-seam-design-and-knowledge-base\n\ndocs: promote GPU seam-design blueprint + route history, add knowledge-base discipline",
          "timestamp": "2026-06-13T11:32:14+08:00",
          "tree_id": "76d13ea6ab4d6dc4c0579717acb972e4c3508ee9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f007991d19cb9c206a4966a0d19133f8838a6762"
        },
        "date": 1781321882608,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 403103.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 627282,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 406376.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 418105,
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
          "id": "064c866e25968157761e3751c93fadd158b56ffd",
          "message": "Merge pull request #128 from LoveDaisy/feat/gpu-single-engine-impl01\n\nfeat(gpu): device-resident continuation engine (§5 Scrum 1)",
          "timestamp": "2026-06-15T09:05:25+08:00",
          "tree_id": "002c927b3387b6839b4d22e45715f46c43bb28f6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/064c866e25968157761e3751c93fadd158b56ffd"
        },
        "date": 1781485857540,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 425850,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608217.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 374601.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 338401.4,
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
          "id": "66f00330c4afa19932534d2a4f993d45d4fd086e",
          "message": "Merge pull request #129 from LoveDaisy/feat/gpu-single-engine-orchestration\n\nfeat(scrum-268): GPU single-engine orchestration — Metal beats legacy in the GUI",
          "timestamp": "2026-06-17T01:11:26+08:00",
          "tree_id": "b8dfda8d13194e60545a1f9f15103718a799d237",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/66f00330c4afa19932534d2a4f993d45d4fd086e"
        },
        "date": 1781630246781,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 318675.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608186.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 406320.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 354745,
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
          "id": "5d0ae08f714a1ffe04c84be055d268ac1a7cf78b",
          "message": "Merge pull request #130 from LoveDaisy/chore/milestone-cleanup\n\nmilestone-cleanup: testing architecture + suite reorg + boundary/doc hardening (scrum-270)",
          "timestamp": "2026-06-17T17:47:17+08:00",
          "tree_id": "5732ebb8dd203a1d9d03c74eff462eadeab90348",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5d0ae08f714a1ffe04c84be055d268ac1a7cf78b"
        },
        "date": 1781690097084,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 302439.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608432.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 374554.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 410681.8,
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
          "id": "44a9f65ea41f3d27b6a3312eaa0400e155dec8a4",
          "message": "Merge pull request #131 from LoveDaisy/perf/metal-gui-optimization\n\nperf: throughput measurement honesty + benchmark scene registry",
          "timestamp": "2026-06-19T15:01:39+08:00",
          "tree_id": "3ad22be58f5edb9734fe8e2cc078a723777417ca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/44a9f65ea41f3d27b6a3312eaa0400e155dec8a4"
        },
        "date": 1781852870927,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 424489.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 626897.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 369615.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 336200,
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
          "id": "51e34348e18bd1fa90c79792dd33c0adf8f4e3ba",
          "message": "Merge pull request #133 from LoveDaisy/fix/geometry-gen-numerical-robustness\n\nfix: extreme-wedge geometry-gen — kill fake basal face (B-ring raypath/subsun anomalies)",
          "timestamp": "2026-06-20T02:19:25+08:00",
          "tree_id": "da51e96d4de912b9c74faa6513b39f7c72ec7720",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51e34348e18bd1fa90c79792dd33c0adf8f4e3ba"
        },
        "date": 1781893504802,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 426053.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 621030.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 373584.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 334875.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "17544d966024fb8652dc9119b52ec05a8644c2fc",
          "message": "chore(deps): bump actions/checkout from 6 to 7\n\nBumps [actions/checkout](https://github.com/actions/checkout) from 6 to 7.\n- [Release notes](https://github.com/actions/checkout/releases)\n- [Changelog](https://github.com/actions/checkout/blob/main/CHANGELOG.md)\n- [Commits](https://github.com/actions/checkout/compare/v6...v7)\n\n---\nupdated-dependencies:\n- dependency-name: actions/checkout\n  dependency-version: '7'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-06-20T08:36:44+08:00",
          "tree_id": "80bfef8b5c3cab32e14340e0a36015d51a662732",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/17544d966024fb8652dc9119b52ec05a8644c2fc"
        },
        "date": 1781916140972,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 433417.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 626868.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 454688.2,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337081.9,
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
          "id": "242e72c6a0abf8e05eb6ac8e298539c4e5308fc6",
          "message": "Merge pull request #135 from LoveDaisy/fix/polygon-face-of-tri-argmax\n\nfix(core): PolygonFaceOfTri argmax for extreme-wedge entry-face mapping",
          "timestamp": "2026-06-20T21:20:27+08:00",
          "tree_id": "cd3d054caf4903ec76fba4d5a0f0b044071670ce",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/242e72c6a0abf8e05eb6ac8e298539c4e5308fc6"
        },
        "date": 1781961997032,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 310804.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611381.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 391856,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 334946.6,
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
          "id": "6241754934a5a559fea3faf320cca1adf7b32c55",
          "message": "Merge pull request #136 from LoveDaisy/chore/land-robustness-doc\n\ndocs: land numerical-robustness conventions doc + AGENTS index (chore 280.1)",
          "timestamp": "2026-06-20T22:57:44+08:00",
          "tree_id": "21a30aa319e31dad0cb42f69d60da88bf99f0708",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6241754934a5a559fea3faf320cca1adf7b32c55"
        },
        "date": 1781967798033,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 414457.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 621889.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 373104,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 341638.7,
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
          "id": "e89453be18b912a0a8d3c630f0c7e330ac2823bd",
          "message": "Merge pull request #137 from LoveDaisy/feat/geometry-numerical-hardening\n\nfix(core/gui): geometry numerical hardening (scrum-280, explore-279 follow-up)",
          "timestamp": "2026-06-21T03:03:02+08:00",
          "tree_id": "3474b4be5cd4c31bdf88716b143bfa112038ef66",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e89453be18b912a0a8d3c630f0c7e330ac2823bd"
        },
        "date": 1781982565584,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 355102.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 624430,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397133.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 322133.7,
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
          "id": "e74ee69ccb36da092f220b1f29f65b376b4e23e2",
          "message": "Merge pull request #138 from LoveDaisy/feat/backend-availability-gui-gate\n\nfeat(gui): runtime backend-availability gate for Metal checkbox",
          "timestamp": "2026-06-21T06:50:12+08:00",
          "tree_id": "64b806e212752d1db31e147b0ce478a9c9c1d06d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e74ee69ccb36da092f220b1f29f65b376b4e23e2"
        },
        "date": 1781996183276,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 425155.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 624130.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 409494.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 331003.8,
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
          "id": "3afcb8103a166e60f969c08f771ce940e87e4834",
          "message": "Merge pull request #139 from LoveDaisy/fix/metal-backend-crash\n\nfix: Metal backend crash on macOS 26.5 — graceful degradation + build-time precompiled metallib",
          "timestamp": "2026-06-22T21:09:18+08:00",
          "tree_id": "c42bf22a1ac73742be973f3c822976d5c4335382",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3afcb8103a166e60f969c08f771ce940e87e4834"
        },
        "date": 1782134160641,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 372766.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 625622.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397412.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 335212.2,
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
          "id": "855e6467599128d7249358a9e7fddd24a1ae9d9d",
          "message": "Merge pull request #140 from LoveDaisy/fix/max-hits-overflow-crash\n\nfix(task-284): max_hits>15 CPU crash (EmplaceBack arena dup) + no-truncation exit-seam",
          "timestamp": "2026-06-23T00:04:58+08:00",
          "tree_id": "e8c0acda0c5c32dd47294ac7226ee4de751f408d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/855e6467599128d7249358a9e7fddd24a1ae9d9d"
        },
        "date": 1782144656842,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 286549.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 604122.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 370710.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 330070.5,
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
          "id": "c814390955ab88969cd00bf92a90d7f81c3773d5",
          "message": "Merge pull request #141 from LoveDaisy/task/env-hardening\n\nEnv-var policy + hardening: centralize knobs, --backend CLI flag, executable gate",
          "timestamp": "2026-06-23T11:01:11+08:00",
          "tree_id": "fe17f04aaff9d0402650e096618af68e19f8e90f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c814390955ab88969cd00bf92a90d7f81c3773d5"
        },
        "date": 1782184058648,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 428354.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608741.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 451284.1,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 329821.3,
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
          "id": "3e0e8e00c10b327af88d8f94b780a29afbdbf592",
          "message": "Merge pull request #142 from LoveDaisy/feat/overlay-refactor\n\nfeat(scrum-288): overlay marker-lines projection completeness + screen-space consistency",
          "timestamp": "2026-06-24T00:10:15+08:00",
          "tree_id": "bca38c2f8909ce5cddc1987abd1ef4f1317c232b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3e0e8e00c10b327af88d8f94b780a29afbdbf592"
        },
        "date": 1782231427058,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 327081.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 607735.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 386654.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 331376.3,
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
          "id": "d61c0aaa76b395c0fe4aaf460569b961e3157bbc",
          "message": "Merge pull request #143 from LoveDaisy/chore/backlog-minor-cleanups\n\nchore: batch of small backlog cleanups (auto-ev refs, deferred review minors, macOS rpath)",
          "timestamp": "2026-06-24T08:55:00+08:00",
          "tree_id": "986d272dc281d1208afcfc7c292929aea99ffa4f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d61c0aaa76b395c0fe4aaf460569b961e3157bbc"
        },
        "date": 1782262844437,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 321316,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 606253.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389003.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 335366.5,
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
          "id": "de095baa6b90e83b8311844f3afca2f828153e09",
          "message": "Merge pull request #144 from LoveDaisy/chore/cleanup-deferred-misc-batch\n\nchore: batch-clean 5 deferred misc backlog items (#292)",
          "timestamp": "2026-06-24T09:59:01+08:00",
          "tree_id": "2432a8f38c538bfa0e3c7455b05eade491dadf3d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de095baa6b90e83b8311844f3afca2f828153e09"
        },
        "date": 1782266698199,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 399244,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610285.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 375586.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 369227.3,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
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
        "date": 1775706601286,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1265332.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1848009,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1196739.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1051067.3,
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
          "id": "1f75c3ab8ea1beb25a0388b63e801e66ef298674",
          "message": "Merge pull request #61 from LoveDaisy/feat/legacy_axis_dist\n\nfeat(core): add legacy Gaussian distribution type",
          "timestamp": "2026-04-09T14:17:22+08:00",
          "tree_id": "3100700029c59b35185531fcee8ac4da29d06826",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1f75c3ab8ea1beb25a0388b63e801e66ef298674"
        },
        "date": 1775715689299,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1102088.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1958492.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1196440.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1048451.8,
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
          "id": "8a3f0bc88f188c376f4274fef6055b64c6500561",
          "message": "Merge pull request #62 from LoveDaisy/fix/ray_alloc\n\nFix ray allocation starvation and C API config limits",
          "timestamp": "2026-04-09T19:30:20+08:00",
          "tree_id": "18ba2e31c99ba6aebf53957871f90c6f0d305fed",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8a3f0bc88f188c376f4274fef6055b64c6500561"
        },
        "date": 1775734442231,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1197594.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1886223.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1196882.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1025328.2,
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
          "id": "7e14e2262bc6720b96deb8ac82df7ff23de37c90",
          "message": "Merge pull request #63 from LoveDaisy/feat/test_coverage\n\ntest: close zero-coverage gaps in optics, c_api, sim_data + fix doc errors",
          "timestamp": "2026-04-10T16:30:44+08:00",
          "tree_id": "be592e1fb54dc3f40eb93d1f98d64bcd28e8a26a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7e14e2262bc6720b96deb8ac82df7ff23de37c90"
        },
        "date": 1775810085904,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 869906.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1882772.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1226027.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 946949.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a80668db54c90087c70fafdb840db88857e9ab07",
          "message": "build(deps): bump actions/download-artifact from 7 to 8 (#64)\n\nBumps [actions/download-artifact](https://github.com/actions/download-artifact) from 7 to 8.\n- [Release notes](https://github.com/actions/download-artifact/releases)\n- [Commits](https://github.com/actions/download-artifact/compare/v7...v8)\n\n---\nupdated-dependencies:\n- dependency-name: actions/download-artifact\n  dependency-version: '8'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>",
          "timestamp": "2026-04-12T20:14:47+08:00",
          "tree_id": "6d94fbf535ea3956bed86891a3a2035db6454167",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a80668db54c90087c70fafdb840db88857e9ab07"
        },
        "date": 1775996353507,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1187726.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1960561,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1243130,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 974878.7,
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
          "id": "024de6454b199b86311707d9ca7f02a104a83f75",
          "message": "Merge pull request #65 from LoveDaisy/feat/test_improve\n\ntest: audit phase 2 — 62 new tests + SampleSphCapPoint rotation fix",
          "timestamp": "2026-04-12T20:23:36+08:00",
          "tree_id": "e79853846edab103efd7e96dad743f4ea39848fe",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/024de6454b199b86311707d9ca7f02a104a83f75"
        },
        "date": 1775996838373,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1129689.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1883875.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1228827.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1044003.2,
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
          "id": "12a30449eb7e798bdfa1f470ae2212f6aa518ead",
          "message": "Merge pull request #66 from LoveDaisy/feat/test_org\n\nReorganize test files: split GUI tests and separate integration tests",
          "timestamp": "2026-04-12T21:53:12+08:00",
          "tree_id": "d528ebc7c90a4396f461148ebfd75a5ede8a3e47",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/12a30449eb7e798bdfa1f470ae2212f6aa518ead"
        },
        "date": 1776002250125,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1154292.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1996745.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1241531.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1054703.4,
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
          "id": "25c74ca230184e57181e36fa4fab523e0623dc7e",
          "message": "Merge pull request #67 from LoveDaisy/feat/ux_tweak\n\nfeat(gui): four UX interaction improvements",
          "timestamp": "2026-04-13T10:27:55+08:00",
          "tree_id": "2f82826d00b92f5318caad5d530ba688bc406bf3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/25c74ca230184e57181e36fa4fab523e0623dc7e"
        },
        "date": 1776047574277,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1043231.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1884446.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1227796.1,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 971792,
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
          "id": "f97dd7cb850710474f5c4580a3e2c57b75814c36",
          "message": "Merge pull request #68 from LoveDaisy/feat/gui_refactor\n\nrefactor(gui): card-based layer/entry layout with copy model",
          "timestamp": "2026-04-14T10:39:15+08:00",
          "tree_id": "ca973ccab53593acaf7fd1bf3fd594e6fd860385",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f97dd7cb850710474f5c4580a3e2c57b75814c36"
        },
        "date": 1776134639387,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 991433.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 2039064.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1225496,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1036201.8,
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
          "id": "ff9fddc949d519efbacf1e341aadf4dd6327f50a",
          "message": "Merge pull request #69 from LoveDaisy/feat/gui_refactor_debt\n\nrefactor(gui): resolve GUI-refactor tech debt + GUI test regressions",
          "timestamp": "2026-04-14T16:31:10+08:00",
          "tree_id": "dc3e20ffaf9b7855bf0448f1dd6042e830aa6ac4",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ff9fddc949d519efbacf1e341aadf4dd6327f50a"
        },
        "date": 1776155756498,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 945488.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1849368.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1210407.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 901100.3,
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
          "id": "104f13eb374ca98ef511f2f3f98d2fa05069feec",
          "message": "Merge pull request #71 from LoveDaisy/feat/gui_polish\n\nGUI polish: edit modal unification, card layout v2, v7 UX fixes",
          "timestamp": "2026-04-19T22:31:08+08:00",
          "tree_id": "5c6045501eb4a202b506b3d96688144625083561",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/104f13eb374ca98ef511f2f3f98d2fa05069feec"
        },
        "date": 1776609314345,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1142410.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1958055.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1242815.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1024188,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6607e4adbb792a86297bb42d03be771efb7495ee",
          "message": "build(deps): bump softprops/action-gh-release from 2 to 3 (#70)\n\nBumps [softprops/action-gh-release](https://github.com/softprops/action-gh-release) from 2 to 3.\n- [Release notes](https://github.com/softprops/action-gh-release/releases)\n- [Changelog](https://github.com/softprops/action-gh-release/blob/master/CHANGELOG.md)\n- [Commits](https://github.com/softprops/action-gh-release/compare/v2...v3)\n\n---\nupdated-dependencies:\n- dependency-name: softprops/action-gh-release\n  dependency-version: '3'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>",
          "timestamp": "2026-04-20T10:51:03+08:00",
          "tree_id": "c8a08fa588e47013e82196eab0275bf7deddf597",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6607e4adbb792a86297bb42d03be771efb7495ee"
        },
        "date": 1776653771192,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1108075.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1955761.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1212561.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 936940.5,
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
          "id": "14383572230a4e449571244b98e6c988a07527b2",
          "message": "Merge pull request #72 from LoveDaisy/feat/gui_polish\n\nfeat(gui): GUI polish — overlay labels, 导出管线重构 & modal 立即模式",
          "timestamp": "2026-04-21T19:02:28+08:00",
          "tree_id": "46011cd81e57f4a345bfd3f6a1eb819208300a17",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/14383572230a4e449571244b98e6c988a07527b2"
        },
        "date": 1776769642829,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1056049.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1919727.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1229362,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 902815.4,
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
          "id": "6ea03e48619368357ee574c62b5c31388d6af037",
          "message": "Merge pull request #73 from LoveDaisy/feat/gui_polish\n\nfeat(gui-polish-v12): 7 子任务 — 滑杆/录屏/Immediate/弹窗布局/face number",
          "timestamp": "2026-04-22T18:41:08+08:00",
          "tree_id": "214de347db4c3436af50e2b7a55d1d805945c514",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6ea03e48619368357ee574c62b5c31388d6af037"
        },
        "date": 1776854757291,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1040615.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 2038049.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1226717.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 681892.9,
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
          "id": "6690144f4a571a7b62131587d5620b8f258c777b",
          "message": "Merge pull request #74 from LoveDaisy/feat/gui_polish\n\nGUI polish: Immediate modal, filter guards, face-number overlay refine",
          "timestamp": "2026-04-23T18:26:03+08:00",
          "tree_id": "dcac66ac084932f8e7a41d73b925f2e1f4abfe50",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6690144f4a571a7b62131587d5620b8f258c777b"
        },
        "date": 1776940236818,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 944118.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1921996.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1226078.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1024436.1,
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
          "id": "e63069a2c407830a197ccf0afe383644959d4b8e",
          "message": "Merge pull request #75 from LoveDaisy/feat/gui_polish\n\nGUI polish v15: orthographic lens, modal H/V layout, detachable Immediate modal",
          "timestamp": "2026-04-24T09:59:54+08:00",
          "tree_id": "568063651932670da5f27b983647c867d4a87abc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e63069a2c407830a197ccf0afe383644959d4b8e"
        },
        "date": 1776996264617,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 997209.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1887091.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1196466.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1029761.7,
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
          "id": "08a2db71b84673557445aaceeeaed31fd09aa279",
          "message": "Merge pull request #76 from LoveDaisy/feat/coordinate_system\n\nCoordinate system overhaul: HaloRay v1 convention + modal preview unification",
          "timestamp": "2026-04-26T13:56:57+08:00",
          "tree_id": "d4bf864cd81160a275162336577ca3c88d1d3bd7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/08a2db71b84673557445aaceeeaed31fd09aa279"
        },
        "date": 1777183272797,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1241832.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1996960.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1508997.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1004700.2,
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
          "id": "cd819bf654497d43842d70955ab1150f677de6a5",
          "message": "Merge pull request #77 from LoveDaisy/dev/misc\n\nchore: misc cleanup and version bump to 4.2.2",
          "timestamp": "2026-04-26T23:44:03+08:00",
          "tree_id": "b57679d8ec11db4aec618fac5ac9a2708914a4ef",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cd819bf654497d43842d70955ab1150f677de6a5"
        },
        "date": 1777218492853,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 853239.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1960303.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1211247.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 979454.1,
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
          "id": "c88a590f41460f51142553918e19cd6da61b6604",
          "message": "Merge pull request #78 from LoveDaisy/feat/gui_polish\n\nfeat(gui): polish overlay labels and orthographic lens behavior",
          "timestamp": "2026-04-27T14:49:09+08:00",
          "tree_id": "49dcf8c39bac9be1da8ffe00b0684430d03fbb09",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c88a590f41460f51142553918e19cd6da61b6604"
        },
        "date": 1777272879148,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 986468.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1920909.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1242022.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 943514.2,
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
          "id": "6878c2d8eca3eb1fcb40efdb7b81fc896a559534",
          "message": "Merge pull request #79 from LoveDaisy/feat/gui_polish\n\nfeat(gui): aspect ratio clamp warning + overlay label sampler refactor",
          "timestamp": "2026-04-28T17:49:24+08:00",
          "tree_id": "5c03cdff293a4b10bd73c61099429e2cbbf1b25e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6878c2d8eca3eb1fcb40efdb7b81fc896a559534"
        },
        "date": 1777370034315,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 908158.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1783024,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1225867.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 985626.6,
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
          "id": "ac272dc408c8e4a5df4ffcbaac9688ce08f0d1ac",
          "message": "Merge pull request #80 from LoveDaisy/feat/doc\n\ndocs: restructure README + add user manual + GUI guide rewrite",
          "timestamp": "2026-04-29T02:54:01+08:00",
          "tree_id": "0358f20d18af1bdf98c12138ee71013d445dc914",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ac272dc408c8e4a5df4ffcbaac9688ce08f0d1ac"
        },
        "date": 1777402706726,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1210406.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1998688.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1195960,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1028401.6,
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
          "id": "26e54569e59d8e18420bf4a3a58d92d268b91b5a",
          "message": "Merge pull request #81 from LoveDaisy/fix/modal_zorder\n\nfix(gui): show combo popups above detached Edit Entry modal",
          "timestamp": "2026-04-29T15:54:43+08:00",
          "tree_id": "6e2ad77922fffeadb6e2da7e47e4fa9f66c34300",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/26e54569e59d8e18420bf4a3a58d92d268b91b5a"
        },
        "date": 1777449561890,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1087669.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1958534.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1240963.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 915337.7,
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
          "id": "620c3717415d7c99e46bf9718a151c931cf669e0",
          "message": "Merge pull request #82 from LoveDaisy/feat/overlay_line_label\n\nfeat(gui): split overlay Line / Label toggles",
          "timestamp": "2026-04-29T17:52:06+08:00",
          "tree_id": "7007374414365f508bf36f92cc6f9453d14e7746",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/620c3717415d7c99e46bf9718a151c931cf669e0"
        },
        "date": 1777456601506,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 941232.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1999659.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1241148.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 975914.9,
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
          "id": "847970a791d9ccde2a8d62467763289fc672771b",
          "message": "Merge pull request #83 from LoveDaisy/feat/globe_view\n\nfeat(gui): Globe lens with trackball, overlay labels, and view reset",
          "timestamp": "2026-04-30T23:23:42+08:00",
          "tree_id": "4c980cbd42b23b410fd148ee16eada28b2feccdb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/847970a791d9ccde2a8d62467763289fc672771b"
        },
        "date": 1777562897022,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 885457.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1887497.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1225662.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1005374.1,
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
          "id": "5c889e1e1f526d6bda40f029153821ff8a06fcf6",
          "message": "Merge pull request #84 from LoveDaisy/feat/ux_crystal_card\n\nfeat(gui): highlight active crystal card while edit modal is open",
          "timestamp": "2026-05-01T00:42:19+08:00",
          "tree_id": "1fe7d7881a3ce16e3cdfd44fdececc046f8f1afd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c889e1e1f526d6bda40f029153821ff8a06fcf6"
        },
        "date": 1777567618046,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1022783.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1921267.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1242611.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1218594,
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
          "id": "efafa43354b35f824c02394d62dbc328464441ba",
          "message": "Merge pull request #85 from LoveDaisy/feat/more_filters\n\nfeat: per-type filter subpanels (Raypath / EntryExit / Direction / Crystal) with .lmc v2 serialization",
          "timestamp": "2026-05-06T11:32:12+08:00",
          "tree_id": "c65cfd36e780f84271a95de1b162d2f55d96cbbf",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/efafa43354b35f824c02394d62dbc328464441ba"
        },
        "date": 1778038620984,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 907536.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1920484.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1242601,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1048891.9,
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
          "id": "f04e1ad447d4f4ccdeef564955f7043422e706e6",
          "message": "Merge pull request #86 from LoveDaisy/feat/globe_view\n\nfeat: Globe↔other lens combo transform, visibility radio buttons, View panel regroup",
          "timestamp": "2026-05-06T12:27:33+08:00",
          "tree_id": "3291df96efef44c52eb9b8d5a8f0c2f7a1722641",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f04e1ad447d4f4ccdeef564955f7043422e706e6"
        },
        "date": 1778041924166,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1040364.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 2083536.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1509957.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 960911.1,
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
          "id": "629b323b216916714ed81f72579bb132517b2088",
          "message": "Merge pull request #87 from LoveDaisy/feat/gui_polish\n\nfeat(gui-polish-v18): visibility row, drop Direction filter, EE align Raypath",
          "timestamp": "2026-05-07T12:18:21+08:00",
          "tree_id": "cba0b9455d12d07982d550091a56b3c28e98d010",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/629b323b216916714ed81f72579bb132517b2088"
        },
        "date": 1778127815601,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1068002.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1959322.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1227350.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1053355.6,
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
          "id": "702fec607e094345d83386ea8011b67d9ccc5718",
          "message": "Merge pull request #88 from LoveDaisy/feat/raypath_symmetry\n\nRaypath P/B/D symmetry redesign + fold/roll fix + e2e regression",
          "timestamp": "2026-05-09T12:44:27+08:00",
          "tree_id": "73a61c0e6a037827a94616b45044f837a5cd9b4f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/702fec607e094345d83386ea8011b67d9ccc5718"
        },
        "date": 1778302157762,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1072492,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1958545.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1210665.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1065624.3,
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
          "id": "940557be35656483ccb5c98a4312d48e7a6b8359",
          "message": "Merge pull request #89 from LoveDaisy/feat/auto_ev\n\nfeat: Adaptive Brightness (P99-anchored auto-EV) for GUI",
          "timestamp": "2026-05-13T09:57:49+08:00",
          "tree_id": "503baee7a3d600f6b16282f53e78474e03be1b8a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/940557be35656483ccb5c98a4312d48e7a6b8359"
        },
        "date": 1778637761364,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1191244.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1813673.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1154035.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1017028.2,
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
          "id": "94e4edf61c07a64640c38d90c1b2dc3fda974653",
          "message": "Merge pull request #90 from LoveDaisy/dev/refactor_c_api\n\nRefactor: enforce GUI/core API boundary via 4 new C API entries",
          "timestamp": "2026-05-14T20:00:37+08:00",
          "tree_id": "99603c8703022d6dc31ab9fa8440052434a0ba84",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/94e4edf61c07a64640c38d90c1b2dc3fda974653"
        },
        "date": 1778760319239,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1187025.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1781948.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1170389.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 906392.1,
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
          "id": "8d97704a2cc32149478a5e9fade9bfca321483d6",
          "message": "Merge pull request #91 from LoveDaisy/feat/update_test_ref\n\nUpdate GUI test references with mean-ref pipeline",
          "timestamp": "2026-05-14T20:18:50+08:00",
          "tree_id": "a2a8ea7ee23efaa4c5c619cec9b8b223fcaf5f5b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d97704a2cc32149478a5e9fade9bfca321483d6"
        },
        "date": 1778761401722,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 817009.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1750745.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1152592.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 964801.4,
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
          "id": "280bf0be7058816e808ba6a78cc98a7a5452eb6b",
          "message": "Merge pull request #92 from LoveDaisy/fix/raypath-1-3\n\nfix(core): repair raypath=[1,3] TIR edge leak via fid_in_src parameter",
          "timestamp": "2026-05-18T08:53:02+08:00",
          "tree_id": "8587ee20db4172496e5a7a3bd75b60244f2d8958",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/280bf0be7058816e808ba6a78cc98a7a5452eb6b"
        },
        "date": 1779065892445,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1130605.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1813378.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1182921.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 960467.4,
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
          "id": "fa8e6cfb354f00e5922cc8d458b330670ff7fb29",
          "message": "Merge pull request #93 from LoveDaisy/feat/query-filter-uplift-v2\n\nfeat(core): uplift query filter to RenderConsumer, fix OFF-mode normalization",
          "timestamp": "2026-05-18T10:04:37+08:00",
          "tree_id": "25b4ddbbd0c316a6ae1f3b02b2309815af457d9c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa8e6cfb354f00e5922cc8d458b330670ff7fb29"
        },
        "date": 1779070174054,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 854360,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1814722.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1181679.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1153863.9,
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
          "id": "b30b552674d681adb8b6123bcbea5954b3340e2f",
          "message": "Merge pull request #94 from LoveDaisy/feat/partition-additivity-test-redesign\n\nRedesign test_partition_buffer_additivity with discriminating metric",
          "timestamp": "2026-05-18T14:17:11+08:00",
          "tree_id": "8c193a6d69f9da7200a7ef970dd3842024d97fb2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b30b552674d681adb8b6123bcbea5954b3340e2f"
        },
        "date": 1779085345553,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 917142.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1817090.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1196366.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 878501.6,
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
          "id": "de1ce7ae90f4d4404c0fc55b92f4a267c52da934",
          "message": "Merge pull request #95 from LoveDaisy/feat/mesh_per_face\n\nfeat(core): scrum-193 leftovers batch — per-face topology + polygon-only tracing",
          "timestamp": "2026-05-18T18:12:13+08:00",
          "tree_id": "e40e53678e278c0e378b085931d4091084ba7f98",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de1ce7ae90f4d4404c0fc55b92f4a267c52da934"
        },
        "date": 1779099455835,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 945749.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1720243.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1091858.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 892384.6,
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
          "id": "5908ce3f2446e604b823aeea72293bb87451f589",
          "message": "Merge pull request #96 from LoveDaisy/dev/refactor_rayseg\n\nrefactor: redesign RaySeg state + sentinel (scrum-206)",
          "timestamp": "2026-05-19T09:32:23+08:00",
          "tree_id": "f209b2da889e6a16d9d82f574c0cc0695d8140f3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5908ce3f2446e604b823aeea72293bb87451f589"
        },
        "date": 1779154652071,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1083639.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1749286,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1154487.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 965832.3,
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
          "id": "c680dcc6a69433924731db037eb416920fd012a4",
          "message": "Merge pull request #97 from LoveDaisy/feat/editor_window_ux\n\nfeat(gui): edit modal UX polish (raypath buffer + pyramid slider alignment)",
          "timestamp": "2026-05-19T11:22:18+08:00",
          "tree_id": "b896575144a36ab9ccd7b0d844d85b5edf0a7929",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c680dcc6a69433924731db037eb416920fd012a4"
        },
        "date": 1779161219625,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1005113.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1781034.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1181417.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 977412.5,
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
          "id": "35e35f3275c09d626a2ebadc49c026d8d790ff17",
          "message": "Merge pull request #98 from LoveDaisy/feat/filter_performance\n\nfeat(core): canonical-form FilterSpec + fix per-ray symmetry hot path",
          "timestamp": "2026-05-20T09:41:53+08:00",
          "tree_id": "fb556e78e23a0fafa29a2be152e168d43f33bfa3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35e35f3275c09d626a2ebadc49c026d8d790ff17"
        },
        "date": 1779241579856,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 781233.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1883238.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1166998.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1161556.7,
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
          "id": "ea5ab5ed603b0df052058857dc75ee0a822d6df4",
          "message": "Merge pull request #99 from LoveDaisy/feat/ux_polish\n\nfeat(gui): ux polish v2 + dead-code cleanup (panels.cpp::SliderWithPreset)",
          "timestamp": "2026-05-20T11:55:13+08:00",
          "tree_id": "286ea92e04c4c205a715bcc8d8867c7166e0bcdd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ea5ab5ed603b0df052058857dc75ee0a822d6df4"
        },
        "date": 1779249586306,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 961081.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1884813,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1129405.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 939077.2,
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
          "id": "2506276291a8c48473a44af1f0e849671d260ff4",
          "message": "Merge pull request #100 from LoveDaisy/fix/capi\n\nfix(c-api): guard sentinel write past max_count + slow regression test",
          "timestamp": "2026-05-20T16:47:27+08:00",
          "tree_id": "64f28a12c3e3144dd31b56a51b4ab9f90d1e4a96",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2506276291a8c48473a44af1f0e849671d260ff4"
        },
        "date": 1779267137965,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1019111.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1815001.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1141946.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 965109.4,
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
          "id": "dec91f357ae70588ef872acc48f9e3268e0d21aa",
          "message": "Merge pull request #101 from LoveDaisy/fix/capi_shared_lib\n\nfix(gui-shared-lib): expose XyzToSrgbUint8 via C API; restore GUI/core boundary",
          "timestamp": "2026-05-20T18:14:57+08:00",
          "tree_id": "5f0da1e0c5f0e2ce6ff584f3de125f66f308436c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dec91f357ae70588ef872acc48f9e3268e0d21aa"
        },
        "date": 1779272420503,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1125049.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1692083.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1138652.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 965386.5,
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
          "id": "135151eed06f1cb0288a1020d0a10c52cc45c485",
          "message": "Merge pull request #102 from LoveDaisy/feat/gui_font\n\nfeat(gui): integrate FontAwesome 6 icon font for 12 GUI buttons",
          "timestamp": "2026-05-21T12:50:01+08:00",
          "tree_id": "949a6505c17485e7d2383f3d8f4f9f6d89603285",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/135151eed06f1cb0288a1020d0a10c52cc45c485"
        },
        "date": 1779339325818,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1196362.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1850166,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1141486.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 829873.3,
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
          "id": "d158a9394600b1ded192e6dc974603f42b76fb47",
          "message": "Merge pull request #103 from LoveDaisy/feat/gui_card_link\n\nfeat(gui): linked entries via ID-pool model + pick-mode UX",
          "timestamp": "2026-05-23T17:41:59+08:00",
          "tree_id": "dcb404af64a0c4302dbf917a45e0f4888b230422",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d158a9394600b1ded192e6dc974603f42b76fb47"
        },
        "date": 1779529615358,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1117924.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1850911.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1140538.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 979250.7,
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
          "id": "dd136e1208d0ad1d81384632592cc2c9421597ab",
          "message": "Merge pull request #104 from LoveDaisy/fix/ms_prob_leak\n\nfix(filter): restore filter as simulator emit-gate (Design A) + architecture doc",
          "timestamp": "2026-05-23T17:55:00+08:00",
          "tree_id": "e624cd047ec6fa17909614f4f428aa2ee3d96f8f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dd136e1208d0ad1d81384632592cc2c9421597ab"
        },
        "date": 1779530384337,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1200024.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1817371.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1182727.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1057918.8,
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
          "id": "e925a47e0db8e111ca3bd397a8ed21c44f8ed48d",
          "message": "Merge pull request #105 from LoveDaisy/fix/adaptive_brightness_screenshot\n\nAdaptive Brightness: single-mode F1 anchor + screenshot fix + P99.5/T135 defaults",
          "timestamp": "2026-05-24T20:37:24+08:00",
          "tree_id": "a7d961fcc9ac82180aab92bb7165fd154a4c4616",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e925a47e0db8e111ca3bd397a8ed21c44f8ed48d"
        },
        "date": 1779626509571,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1159741.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1850567.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1212230.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 965700.8,
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
          "id": "fc3e2fa37ad9f4bce408a9a5817f09eb57aa7c8a",
          "message": "Merge pull request #106 from LoveDaisy/feat/gui_zenith_nadir\n\nfeat(gui): zenith/nadir ring marker overlay",
          "timestamp": "2026-05-25T01:23:44+08:00",
          "tree_id": "3f5880b3f64c960c7f36c1b0c568f0b38f820075",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc3e2fa37ad9f4bce408a9a5817f09eb57aa7c8a"
        },
        "date": 1779643718942,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1150547,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1816925.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1169024.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 903378.6,
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
          "id": "22dc93834aba6b5b8c0e55d16027165f830004f8",
          "message": "Merge pull request #107 from LoveDaisy/feat/ci_test\n\nci: add slow e2e test job with shared-lib build",
          "timestamp": "2026-05-25T09:25:31+08:00",
          "tree_id": "8bdc09a9c9a6566dab36fe1e61008d26ef894aab",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/22dc93834aba6b5b8c0e55d16027165f830004f8"
        },
        "date": 1779672708856,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1209656.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1817301.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1182916.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 908513.9,
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
          "id": "e6da1a081780ca1412d671fc3108615df715b988",
          "message": "Merge pull request #108 from LoveDaisy/fix/ms_filter_leak\n\nfix(simulator): prevent MS filter-leaked rays from reaching main output",
          "timestamp": "2026-05-25T14:17:56+08:00",
          "tree_id": "f0ddc6dd281209f960a42d9ba62a62b1e4389732",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e6da1a081780ca1412d671fc3108615df715b988"
        },
        "date": 1779690121073,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1178917.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1885618.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1197820,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1235343.1,
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
          "id": "b555bce0cfb41ac2959a09682690abaee59e4dcc",
          "message": "Merge pull request #109 from LoveDaisy/dev/env_var\n\nrefactor: replace LUMICE_SIM_SEED env var with C API sim_seed parameter",
          "timestamp": "2026-05-25T15:12:52+08:00",
          "tree_id": "7b0e4e2ac9d8d9d06a5c565821e5ba98db920649",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b555bce0cfb41ac2959a09682690abaee59e4dcc"
        },
        "date": 1779693464905,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1176265.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1851023.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1214249.7,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1010644.1,
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
          "id": "57735b3bf85ff7ae78b102ea0d93aca76e34cc73",
          "message": "Merge pull request #110 from LoveDaisy/feat/render_refresh\n\nFix rendering update issues, simulator audit gaps, and raise max hits",
          "timestamp": "2026-05-26T01:05:00+08:00",
          "tree_id": "9b9aab40e5072b0b7c0942bdb8ed63184483a299",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/57735b3bf85ff7ae78b102ea0d93aca76e34cc73"
        },
        "date": 1779728986605,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1093848.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1784920.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1128505.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 886347.3,
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
          "id": "9db583df928264e5fe43a90e364da883ed197704",
          "message": "Merge pull request #111 from LoveDaisy/feat/gui_minor\n\nfeat(gui): card-wide click and front-hemisphere checkbox",
          "timestamp": "2026-05-27T08:18:34+08:00",
          "tree_id": "3010b3aeccd240cd41893b564209d19ca5731165",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9db583df928264e5fe43a90e364da883ed197704"
        },
        "date": 1779841359874,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1156253.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1722239.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1116907.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1127536.9,
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
          "id": "062fff11d9ca5a48e9424fc2424572c415b9460f",
          "message": "Merge pull request #112 from LoveDaisy/fix/misc\n\nchore(gui): unify overlay sentinel and add review follow-up fixes",
          "timestamp": "2026-05-27T08:42:42+08:00",
          "tree_id": "adcb4da74d7fbf0987da6ef3c60a107b7c34e64a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/062fff11d9ca5a48e9424fc2424572c415b9460f"
        },
        "date": 1779842847258,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1118454.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1818287.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1117299.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 830599.1,
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
          "id": "eb70214863932c839b9f6102a4ce5961856333bc",
          "message": "Merge pull request #113 from LoveDaisy/chore/gui_lag\n\nfix(gui): eliminate high-resolution GUI lag",
          "timestamp": "2026-05-27T10:08:15+08:00",
          "tree_id": "a7433ce2d4a833d2ea80f18d110d7ea3ce15f183",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/eb70214863932c839b9f6102a4ce5961856333bc"
        },
        "date": 1779847934634,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1091982.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1922352.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1128713.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1475354.5,
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
          "id": "e4a3f6587b8ef382e420de0b5d739f4f5bc0d8c5",
          "message": "Merge pull request #114 from LoveDaisy/chore/major_audit\n\nchore: major audit — architecture docs, tooltips, and cross-ref comments",
          "timestamp": "2026-05-28T08:33:03+08:00",
          "tree_id": "aabe60b8c5d31e9c2a26d6fc057ffc6fbfe12dad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e4a3f6587b8ef382e420de0b5d739f4f5bc0d8c5"
        },
        "date": 1779928709372,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1119980.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1818180.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1105639.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 869829,
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
          "id": "e3d86034ece73c9d5b32a9981fc5b324bdbfbce6",
          "message": "Merge pull request #115 from LoveDaisy/feat/remove_ev_anchor\n\nRemove F1 anchor lane, use self-P99.5 EV normalization",
          "timestamp": "2026-05-28T15:38:16+08:00",
          "tree_id": "52693d70e631b7a71599cdcad16ec33dd9f6c0d1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e3d86034ece73c9d5b32a9981fc5b324bdbfbce6"
        },
        "date": 1779954198607,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1145463.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1886299.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1068044.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 936650.1,
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
          "id": "dc2918aa9d575eeb2b3007788b1bb8cd62f29193",
          "message": "Merge pull request #116 from LoveDaisy/feat/ev-anchor-p99\n\nfeat(auto-ev): rework adaptive brightness anchor (P99 + global f=8 downsample metric)",
          "timestamp": "2026-05-31T20:02:04+08:00",
          "tree_id": "cd1482b671d823bc75c304687bebc49aaa388c11",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dc2918aa9d575eeb2b3007788b1bb8cd62f29193"
        },
        "date": 1780229218452,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 785529.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1722264.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1080530.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 898519.1,
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
          "id": "dfad7e8b454dfdd7903349cf553465cdcacf91e9",
          "message": "Merge pull request #117 from LoveDaisy/chore/ev-pipeline-doc-sync\n\ndocs: align EV/filter architecture docs to single-lane (post anchor-lane removal)",
          "timestamp": "2026-06-01T00:01:46+08:00",
          "tree_id": "bcc818e5077e221eaa5599a098d72d4c7e60f479",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfad7e8b454dfdd7903349cf553465cdcacf91e9"
        },
        "date": 1780243627839,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1120342.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1752454.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1127290.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 946607,
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
          "id": "d726d4f8d498793da4007ea5f1b139f12c7d5a9d",
          "message": "Merge pull request #118 from LoveDaisy/feat/ee-filter-uplift\n\nfeat: EntryExit filter expressiveness — wildcard, multi-value OR, length bounds",
          "timestamp": "2026-06-01T12:57:04+08:00",
          "tree_id": "2d104ee146202faca56b9fd9ed006a45cf215c55",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d726d4f8d498793da4007ea5f1b139f12c7d5a9d"
        },
        "date": 1780290113705,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 871339.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1816094.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1104229.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 945630.6,
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
          "id": "a83345b608ebc03c14a674ba2d9e86a766c27e61",
          "message": "Merge pull request #119 from LoveDaisy/dev/soa_refactor\n\nCPU SoA: ray recorder footprint optimization (RaypathRecorder split + SBO)",
          "timestamp": "2026-06-02T14:44:34+08:00",
          "tree_id": "54bf6808db99db93563899e56f190d727a9b2868",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a83345b608ebc03c14a674ba2d9e86a766c27e61"
        },
        "date": 1780382992353,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 954438.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1817598.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1404924.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 880318.9,
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
          "id": "392ee041c7255f883dcb83f6eb4c4958ea8f8d28",
          "message": "Merge pull request #120 from LoveDaisy/dev/perf_opt2\n\nperf(worker-default): default worker count to physical cores",
          "timestamp": "2026-06-03T18:05:54+08:00",
          "tree_id": "0863f2e30f743b4cf8bdb1004f290e0da8e43762",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/392ee041c7255f883dcb83f6eb4c4958ea8f8d28"
        },
        "date": 1780481470070,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 999278.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1235056.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 784635.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 635228.5,
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
          "id": "390813463050266f30c6406eab1cb4b9a52972bb",
          "message": "Merge pull request #121 from LoveDaisy/feat/metal_backend_prod\n\nfeat: Metal trace backend (production) — pluggable TraceBackend seam + CPU/Metal parity",
          "timestamp": "2026-06-08T17:19:20+08:00",
          "tree_id": "ce38b259f1a458387effaafcbece2fdfa5c2e196",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/390813463050266f30c6406eab1cb4b9a52972bb"
        },
        "date": 1780910679408,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1179938.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1250335.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 803546.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 635745.8,
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
          "id": "5c588072df470acedf8b0eb419f2c1b278e77239",
          "message": "Merge pull request #122 from LoveDaisy/feat/metal-gui-default\n\nfeat: enable Metal backend on GUI live-preview (dual-fisheye-EA + C API toggle)",
          "timestamp": "2026-06-09T09:33:36+08:00",
          "tree_id": "d8c695861215eb9ce11c1bf5c993c7a78a99cece",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c588072df470acedf8b0eb419f2c1b278e77239"
        },
        "date": 1780969099606,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1181300.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1235025.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 754754.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 639947.5,
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
          "id": "ca327679e29ff539e1ab7a1b51177ed6066af02f",
          "message": "Merge pull request #123 from LoveDaisy/feat/metal-exit-seam\n\nfeat(metal): exit-seam buffer egress 出口替代整幅回读 (P1/scrum-258)",
          "timestamp": "2026-06-12T09:45:24+08:00",
          "tree_id": "0af735a07c022e2c17115e13424724ac9d00cca0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ca327679e29ff539e1ab7a1b51177ed6066af02f"
        },
        "date": 1781229034465,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1007267.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1266354.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 803558.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 626443.3,
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
          "id": "c5b5985c34ddd97e7372b393a1d7892279bf136c",
          "message": "Merge pull request #124 from LoveDaisy/feat/metal-rootgen\n\nfeat(metal): device GPU root-gen 默认根供给 + 吞吐确证 (P2/scrum-260+261)",
          "timestamp": "2026-06-12T10:11:44+08:00",
          "tree_id": "7e1566edab98bcbd5054d82dd2e28bbc19b99c7e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c5b5985c34ddd97e7372b393a1d7892279bf136c"
        },
        "date": 1781230753905,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1189759.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1266307.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 749117.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 678815.9,
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
          "id": "207de068d30e8080608200a0b1cf9b54c3518f43",
          "message": "Merge pull request #125 from LoveDaisy/fix/serverimpl-stop-deadlock\n\nfix: ServerImpl::Stop() condition-variable lost-wakeup deadlock + stress regression",
          "timestamp": "2026-06-12T11:53:36+08:00",
          "tree_id": "476d50f4d2d83297176fb1ddb991155d55897e02",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/207de068d30e8080608200a0b1cf9b54c3518f43"
        },
        "date": 1781236763616,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1154237.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1250441.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 754783.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 680840.6,
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
          "id": "00f9d49fc95fe411dd82ab6686e21fc0de5718e2",
          "message": "Merge pull request #126 from LoveDaisy/feat/metal-gen-trace-fusion\n\nfeat(metal): fuse gen+trace into one command buffer (task-264)",
          "timestamp": "2026-06-13T01:45:22+08:00",
          "tree_id": "9aed83822638731d531d0ae74a7dc2df39d40d92",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/00f9d49fc95fe411dd82ab6686e21fc0de5718e2"
        },
        "date": 1781286655566,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1033395.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1250487.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 797045.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 636885.2,
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
          "id": "f007991d19cb9c206a4966a0d19133f8838a6762",
          "message": "Merge pull request #127 from LoveDaisy/docs/gpu-seam-design-and-knowledge-base\n\ndocs: promote GPU seam-design blueprint + route history, add knowledge-base discipline",
          "timestamp": "2026-06-13T11:32:14+08:00",
          "tree_id": "76d13ea6ab4d6dc4c0579717acb972e4c3508ee9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f007991d19cb9c206a4966a0d19133f8838a6762"
        },
        "date": 1781321886937,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1046318.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1266352.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 790774,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 818983.2,
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
          "id": "064c866e25968157761e3751c93fadd158b56ffd",
          "message": "Merge pull request #128 from LoveDaisy/feat/gpu-single-engine-impl01\n\nfeat(gpu): device-resident continuation engine (§5 Scrum 1)",
          "timestamp": "2026-06-15T09:05:25+08:00",
          "tree_id": "002c927b3387b6839b4d22e45715f46c43bb28f6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/064c866e25968157761e3751c93fadd158b56ffd"
        },
        "date": 1781485861414,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1179315.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1250545.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 749128.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 633064.5,
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
          "id": "66f00330c4afa19932534d2a4f993d45d4fd086e",
          "message": "Merge pull request #129 from LoveDaisy/feat/gpu-single-engine-orchestration\n\nfeat(scrum-268): GPU single-engine orchestration — Metal beats legacy in the GUI",
          "timestamp": "2026-06-17T01:11:26+08:00",
          "tree_id": "b8dfda8d13194e60545a1f9f15103718a799d237",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/66f00330c4afa19932534d2a4f993d45d4fd086e"
        },
        "date": 1781630249780,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 917850.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1250415.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 797098.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 678031.8,
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
          "id": "5d0ae08f714a1ffe04c84be055d268ac1a7cf78b",
          "message": "Merge pull request #130 from LoveDaisy/chore/milestone-cleanup\n\nmilestone-cleanup: testing architecture + suite reorg + boundary/doc hardening (scrum-270)",
          "timestamp": "2026-06-17T17:47:17+08:00",
          "tree_id": "5732ebb8dd203a1d9d03c74eff462eadeab90348",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5d0ae08f714a1ffe04c84be055d268ac1a7cf78b"
        },
        "date": 1781690101064,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1029055.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1266352.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 749063.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 753328.6,
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
          "id": "44a9f65ea41f3d27b6a3312eaa0400e155dec8a4",
          "message": "Merge pull request #131 from LoveDaisy/perf/metal-gui-optimization\n\nperf: throughput measurement honesty + benchmark scene registry",
          "timestamp": "2026-06-19T15:01:39+08:00",
          "tree_id": "3ad22be58f5edb9734fe8e2cc078a723777417ca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/44a9f65ea41f3d27b6a3312eaa0400e155dec8a4"
        },
        "date": 1781852874163,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 851257.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1266809.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 676814.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 633673.8,
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
          "id": "51e34348e18bd1fa90c79792dd33c0adf8f4e3ba",
          "message": "Merge pull request #133 from LoveDaisy/fix/geometry-gen-numerical-robustness\n\nfix: extreme-wedge geometry-gen — kill fake basal face (B-ring raypath/subsun anomalies)",
          "timestamp": "2026-06-20T02:19:25+08:00",
          "tree_id": "da51e96d4de912b9c74faa6513b39f7c72ec7720",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51e34348e18bd1fa90c79792dd33c0adf8f4e3ba"
        },
        "date": 1781893507765,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 888087.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1260373,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 665673.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 630332.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "17544d966024fb8652dc9119b52ec05a8644c2fc",
          "message": "chore(deps): bump actions/checkout from 6 to 7\n\nBumps [actions/checkout](https://github.com/actions/checkout) from 6 to 7.\n- [Release notes](https://github.com/actions/checkout/releases)\n- [Changelog](https://github.com/actions/checkout/blob/main/CHANGELOG.md)\n- [Commits](https://github.com/actions/checkout/compare/v6...v7)\n\n---\nupdated-dependencies:\n- dependency-name: actions/checkout\n  dependency-version: '7'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-06-20T08:36:44+08:00",
          "tree_id": "80bfef8b5c3cab32e14340e0a36015d51a662732",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/17544d966024fb8652dc9119b52ec05a8644c2fc"
        },
        "date": 1781916143839,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 861573.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1254234,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 775302.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 632904.6,
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
          "id": "242e72c6a0abf8e05eb6ac8e298539c4e5308fc6",
          "message": "Merge pull request #135 from LoveDaisy/fix/polygon-face-of-tri-argmax\n\nfix(core): PolygonFaceOfTri argmax for extreme-wedge entry-face mapping",
          "timestamp": "2026-06-20T21:20:27+08:00",
          "tree_id": "cd3d054caf4903ec76fba4d5a0f0b044071670ce",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/242e72c6a0abf8e05eb6ac8e298539c4e5308fc6"
        },
        "date": 1781961999556,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 639758.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1258398.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 712709.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 636141,
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
          "id": "6241754934a5a559fea3faf320cca1adf7b32c55",
          "message": "Merge pull request #136 from LoveDaisy/chore/land-robustness-doc\n\ndocs: land numerical-robustness conventions doc + AGENTS index (chore 280.1)",
          "timestamp": "2026-06-20T22:57:44+08:00",
          "tree_id": "21a30aa319e31dad0cb42f69d60da88bf99f0708",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6241754934a5a559fea3faf320cca1adf7b32c55"
        },
        "date": 1781967801021,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 816886.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1258672.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 665258.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 638405.1,
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
          "id": "e89453be18b912a0a8d3c630f0c7e330ac2823bd",
          "message": "Merge pull request #137 from LoveDaisy/feat/geometry-numerical-hardening\n\nfix(core/gui): geometry numerical hardening (scrum-280, explore-279 follow-up)",
          "timestamp": "2026-06-21T03:03:02+08:00",
          "tree_id": "3474b4be5cd4c31bdf88716b143bfa112038ef66",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e89453be18b912a0a8d3c630f0c7e330ac2823bd"
        },
        "date": 1781982568170,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 717786.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1251666.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 699177.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 592285.4,
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
          "id": "e74ee69ccb36da092f220b1f29f65b376b4e23e2",
          "message": "Merge pull request #138 from LoveDaisy/feat/backend-availability-gui-gate\n\nfeat(gui): runtime backend-availability gate for Metal checkbox",
          "timestamp": "2026-06-21T06:50:12+08:00",
          "tree_id": "64b806e212752d1db31e147b0ce478a9c9c1d06d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e74ee69ccb36da092f220b1f29f65b376b4e23e2"
        },
        "date": 1781996185841,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 939525.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1251497.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 738580.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 626433.9,
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
          "id": "3afcb8103a166e60f969c08f771ce940e87e4834",
          "message": "Merge pull request #139 from LoveDaisy/fix/metal-backend-crash\n\nfix: Metal backend crash on macOS 26.5 — graceful degradation + build-time precompiled metallib",
          "timestamp": "2026-06-22T21:09:18+08:00",
          "tree_id": "c42bf22a1ac73742be973f3c822976d5c4335382",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3afcb8103a166e60f969c08f771ce940e87e4834"
        },
        "date": 1782134163817,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 800629.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1237356.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 695462,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 622234.7,
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
          "id": "855e6467599128d7249358a9e7fddd24a1ae9d9d",
          "message": "Merge pull request #140 from LoveDaisy/fix/max-hits-overflow-crash\n\nfix(task-284): max_hits>15 CPU crash (EmplaceBack arena dup) + no-truncation exit-seam",
          "timestamp": "2026-06-23T00:04:58+08:00",
          "tree_id": "e8c0acda0c5c32dd47294ac7226ee4de751f408d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/855e6467599128d7249358a9e7fddd24a1ae9d9d"
        },
        "date": 1782144661467,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 568239.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1216615,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 684219.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 583540.7,
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
          "id": "c814390955ab88969cd00bf92a90d7f81c3773d5",
          "message": "Merge pull request #141 from LoveDaisy/task/env-hardening\n\nEnv-var policy + hardening: centralize knobs, --backend CLI flag, executable gate",
          "timestamp": "2026-06-23T11:01:11+08:00",
          "tree_id": "fe17f04aaff9d0402650e096618af68e19f8e90f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c814390955ab88969cd00bf92a90d7f81c3773d5"
        },
        "date": 1782184062618,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 873051.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1245435.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 756210,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 623324.3,
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
          "id": "3e0e8e00c10b327af88d8f94b780a29afbdbf592",
          "message": "Merge pull request #142 from LoveDaisy/feat/overlay-refactor\n\nfeat(scrum-288): overlay marker-lines projection completeness + screen-space consistency",
          "timestamp": "2026-06-24T00:10:15+08:00",
          "tree_id": "bca38c2f8909ce5cddc1987abd1ef4f1317c232b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3e0e8e00c10b327af88d8f94b780a29afbdbf592"
        },
        "date": 1782231430063,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 595219,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1217519.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 724601.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 594506.7,
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
          "id": "d61c0aaa76b395c0fe4aaf460569b961e3157bbc",
          "message": "Merge pull request #143 from LoveDaisy/chore/backlog-minor-cleanups\n\nchore: batch of small backlog cleanups (auto-ev refs, deferred review minors, macOS rpath)",
          "timestamp": "2026-06-24T08:55:00+08:00",
          "tree_id": "986d272dc281d1208afcfc7c292929aea99ffa4f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d61c0aaa76b395c0fe4aaf460569b961e3157bbc"
        },
        "date": 1782262847715,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 791881.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1232482.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 704959.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 628134.2,
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
          "id": "de095baa6b90e83b8311844f3afca2f828153e09",
          "message": "Merge pull request #144 from LoveDaisy/chore/cleanup-deferred-misc-batch\n\nchore: batch-clean 5 deferred misc backlog items (#292)",
          "timestamp": "2026-06-24T09:59:01+08:00",
          "tree_id": "2432a8f38c538bfa0e3c7455b05eade491dadf3d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de095baa6b90e83b8311844f3afca2f828153e09"
        },
        "date": 1782266701390,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 854239.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1247914.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 674214.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 632914.9,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
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
        "date": 1775706603030,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 90.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 73.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 75.4,
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
          "id": "1f75c3ab8ea1beb25a0388b63e801e66ef298674",
          "message": "Merge pull request #61 from LoveDaisy/feat/legacy_axis_dist\n\nfeat(core): add legacy Gaussian distribution type",
          "timestamp": "2026-04-09T14:17:22+08:00",
          "tree_id": "3100700029c59b35185531fcee8ac4da29d06826",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1f75c3ab8ea1beb25a0388b63e801e66ef298674"
        },
        "date": 1775715691149,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 96.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 68,
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
          "id": "8a3f0bc88f188c376f4274fef6055b64c6500561",
          "message": "Merge pull request #62 from LoveDaisy/fix/ray_alloc\n\nFix ray allocation starvation and C API config limits",
          "timestamp": "2026-04-09T19:30:20+08:00",
          "tree_id": "18ba2e31c99ba6aebf53957871f90c6f0d305fed",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8a3f0bc88f188c376f4274fef6055b64c6500561"
        },
        "date": 1775734444068,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 91.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 75.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 57.8,
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
          "id": "7e14e2262bc6720b96deb8ac82df7ff23de37c90",
          "message": "Merge pull request #63 from LoveDaisy/feat/test_coverage\n\ntest: close zero-coverage gaps in optics, c_api, sim_data + fix doc errors",
          "timestamp": "2026-04-10T16:30:44+08:00",
          "tree_id": "be592e1fb54dc3f40eb93d1f98d64bcd28e8a26a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7e14e2262bc6720b96deb8ac82df7ff23de37c90"
        },
        "date": 1775810087022,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 67.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a80668db54c90087c70fafdb840db88857e9ab07",
          "message": "build(deps): bump actions/download-artifact from 7 to 8 (#64)\n\nBumps [actions/download-artifact](https://github.com/actions/download-artifact) from 7 to 8.\n- [Release notes](https://github.com/actions/download-artifact/releases)\n- [Commits](https://github.com/actions/download-artifact/compare/v7...v8)\n\n---\nupdated-dependencies:\n- dependency-name: actions/download-artifact\n  dependency-version: '8'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>",
          "timestamp": "2026-04-12T20:14:47+08:00",
          "tree_id": "6d94fbf535ea3956bed86891a3a2035db6454167",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a80668db54c90087c70fafdb840db88857e9ab07"
        },
        "date": 1775996355075,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 59.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 69.4,
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
          "id": "024de6454b199b86311707d9ca7f02a104a83f75",
          "message": "Merge pull request #65 from LoveDaisy/feat/test_improve\n\ntest: audit phase 2 — 62 new tests + SampleSphCapPoint rotation fix",
          "timestamp": "2026-04-12T20:23:36+08:00",
          "tree_id": "e79853846edab103efd7e96dad743f4ea39848fe",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/024de6454b199b86311707d9ca7f02a104a83f75"
        },
        "date": 1775996839423,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 72.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 61.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 70.5,
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
          "id": "12a30449eb7e798bdfa1f470ae2212f6aa518ead",
          "message": "Merge pull request #66 from LoveDaisy/feat/test_org\n\nReorganize test files: split GUI tests and separate integration tests",
          "timestamp": "2026-04-12T21:53:12+08:00",
          "tree_id": "d528ebc7c90a4396f461148ebfd75a5ede8a3e47",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/12a30449eb7e798bdfa1f470ae2212f6aa518ead"
        },
        "date": 1776002251466,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 95.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 69.3,
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
          "id": "25c74ca230184e57181e36fa4fab523e0623dc7e",
          "message": "Merge pull request #67 from LoveDaisy/feat/ux_tweak\n\nfeat(gui): four UX interaction improvements",
          "timestamp": "2026-04-13T10:27:55+08:00",
          "tree_id": "2f82826d00b92f5318caad5d530ba688bc406bf3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/25c74ca230184e57181e36fa4fab523e0623dc7e"
        },
        "date": 1776047576023,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 92.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 60.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 71.3,
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
          "id": "f97dd7cb850710474f5c4580a3e2c57b75814c36",
          "message": "Merge pull request #68 from LoveDaisy/feat/gui_refactor\n\nrefactor(gui): card-based layer/entry layout with copy model",
          "timestamp": "2026-04-14T10:39:15+08:00",
          "tree_id": "ca973ccab53593acaf7fd1bf3fd594e6fd860385",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f97dd7cb850710474f5c4580a3e2c57b75814c36"
        },
        "date": 1776134640636,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 78.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 68.1,
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
          "id": "ff9fddc949d519efbacf1e341aadf4dd6327f50a",
          "message": "Merge pull request #69 from LoveDaisy/feat/gui_refactor_debt\n\nrefactor(gui): resolve GUI-refactor tech debt + GUI test regressions",
          "timestamp": "2026-04-14T16:31:10+08:00",
          "tree_id": "dc3e20ffaf9b7855bf0448f1dd6042e830aa6ac4",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ff9fddc949d519efbacf1e341aadf4dd6327f50a"
        },
        "date": 1776155758416,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 73.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 66.3,
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
          "id": "104f13eb374ca98ef511f2f3f98d2fa05069feec",
          "message": "Merge pull request #71 from LoveDaisy/feat/gui_polish\n\nGUI polish: edit modal unification, card layout v2, v7 UX fixes",
          "timestamp": "2026-04-19T22:31:08+08:00",
          "tree_id": "5c6045501eb4a202b506b3d96688144625083561",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/104f13eb374ca98ef511f2f3f98d2fa05069feec"
        },
        "date": 1776609315369,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 78,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 67.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6607e4adbb792a86297bb42d03be771efb7495ee",
          "message": "build(deps): bump softprops/action-gh-release from 2 to 3 (#70)\n\nBumps [softprops/action-gh-release](https://github.com/softprops/action-gh-release) from 2 to 3.\n- [Release notes](https://github.com/softprops/action-gh-release/releases)\n- [Changelog](https://github.com/softprops/action-gh-release/blob/master/CHANGELOG.md)\n- [Commits](https://github.com/softprops/action-gh-release/compare/v2...v3)\n\n---\nupdated-dependencies:\n- dependency-name: softprops/action-gh-release\n  dependency-version: '3'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>\nCo-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>",
          "timestamp": "2026-04-20T10:51:03+08:00",
          "tree_id": "c8a08fa588e47013e82196eab0275bf7deddf597",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6607e4adbb792a86297bb42d03be771efb7495ee"
        },
        "date": 1776653772781,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 100.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.6,
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
          "id": "14383572230a4e449571244b98e6c988a07527b2",
          "message": "Merge pull request #72 from LoveDaisy/feat/gui_polish\n\nfeat(gui): GUI polish — overlay labels, 导出管线重构 & modal 立即模式",
          "timestamp": "2026-04-21T19:02:28+08:00",
          "tree_id": "46011cd81e57f4a345bfd3f6a1eb819208300a17",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/14383572230a4e449571244b98e6c988a07527b2"
        },
        "date": 1776769644606,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 90,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 60.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 60.3,
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
          "id": "6ea03e48619368357ee574c62b5c31388d6af037",
          "message": "Merge pull request #73 from LoveDaisy/feat/gui_polish\n\nfeat(gui-polish-v12): 7 子任务 — 滑杆/录屏/Immediate/弹窗布局/face number",
          "timestamp": "2026-04-22T18:41:08+08:00",
          "tree_id": "214de347db4c3436af50e2b7a55d1d805945c514",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6ea03e48619368357ee574c62b5c31388d6af037"
        },
        "date": 1776854759628,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 93.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 81.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 50.5,
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
          "id": "6690144f4a571a7b62131587d5620b8f258c777b",
          "message": "Merge pull request #74 from LoveDaisy/feat/gui_polish\n\nGUI polish: Immediate modal, filter guards, face-number overlay refine",
          "timestamp": "2026-04-23T18:26:03+08:00",
          "tree_id": "dcac66ac084932f8e7a41d73b925f2e1f4abfe50",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6690144f4a571a7b62131587d5620b8f258c777b"
        },
        "date": 1776940238884,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 101.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 67.9,
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
          "id": "e63069a2c407830a197ccf0afe383644959d4b8e",
          "message": "Merge pull request #75 from LoveDaisy/feat/gui_polish\n\nGUI polish v15: orthographic lens, modal H/V layout, detachable Immediate modal",
          "timestamp": "2026-04-24T09:59:54+08:00",
          "tree_id": "568063651932670da5f27b983647c867d4a87abc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e63069a2c407830a197ccf0afe383644959d4b8e"
        },
        "date": 1776996266014,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 94,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 72.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 75.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 67.1,
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
          "id": "08a2db71b84673557445aaceeeaed31fd09aa279",
          "message": "Merge pull request #76 from LoveDaisy/feat/coordinate_system\n\nCoordinate system overhaul: HaloRay v1 convention + modal preview unification",
          "timestamp": "2026-04-26T13:56:57+08:00",
          "tree_id": "d4bf864cd81160a275162336577ca3c88d1d3bd7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/08a2db71b84673557445aaceeeaed31fd09aa279"
        },
        "date": 1777183275333,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.7,
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
          "id": "cd819bf654497d43842d70955ab1150f677de6a5",
          "message": "Merge pull request #77 from LoveDaisy/dev/misc\n\nchore: misc cleanup and version bump to 4.2.2",
          "timestamp": "2026-04-26T23:44:03+08:00",
          "tree_id": "b57679d8ec11db4aec618fac5ac9a2708914a4ef",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cd819bf654497d43842d70955ab1150f677de6a5"
        },
        "date": 1777218494834,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 72.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 59.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 72.3,
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
          "id": "c88a590f41460f51142553918e19cd6da61b6604",
          "message": "Merge pull request #78 from LoveDaisy/feat/gui_polish\n\nfeat(gui): polish overlay labels and orthographic lens behavior",
          "timestamp": "2026-04-27T14:49:09+08:00",
          "tree_id": "49dcf8c39bac9be1da8ffe00b0684430d03fbb09",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c88a590f41460f51142553918e19cd6da61b6604"
        },
        "date": 1777272881422,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 92.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 76.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 70.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 63,
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
          "id": "6878c2d8eca3eb1fcb40efdb7b81fc896a559534",
          "message": "Merge pull request #79 from LoveDaisy/feat/gui_polish\n\nfeat(gui): aspect ratio clamp warning + overlay label sampler refactor",
          "timestamp": "2026-04-28T17:49:24+08:00",
          "tree_id": "5c03cdff293a4b10bd73c61099429e2cbbf1b25e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6878c2d8eca3eb1fcb40efdb7b81fc896a559534"
        },
        "date": 1777370036515,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 96.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 68.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 57.1,
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
          "id": "ac272dc408c8e4a5df4ffcbaac9688ce08f0d1ac",
          "message": "Merge pull request #80 from LoveDaisy/feat/doc\n\ndocs: restructure README + add user manual + GUI guide rewrite",
          "timestamp": "2026-04-29T02:54:01+08:00",
          "tree_id": "0358f20d18af1bdf98c12138ee71013d445dc914",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ac272dc408c8e4a5df4ffcbaac9688ce08f0d1ac"
        },
        "date": 1777402708312,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77.1,
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
          "id": "26e54569e59d8e18420bf4a3a58d92d268b91b5a",
          "message": "Merge pull request #81 from LoveDaisy/fix/modal_zorder\n\nfix(gui): show combo popups above detached Edit Entry modal",
          "timestamp": "2026-04-29T15:54:43+08:00",
          "tree_id": "6e2ad77922fffeadb6e2da7e47e4fa9f66c34300",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/26e54569e59d8e18420bf4a3a58d92d268b91b5a"
        },
        "date": 1777449563524,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 80.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 60.8,
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
          "id": "620c3717415d7c99e46bf9718a151c931cf669e0",
          "message": "Merge pull request #82 from LoveDaisy/feat/overlay_line_label\n\nfeat(gui): split overlay Line / Label toggles",
          "timestamp": "2026-04-29T17:52:06+08:00",
          "tree_id": "7007374414365f508bf36f92cc6f9453d14e7746",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/620c3717415d7c99e46bf9718a151c931cf669e0"
        },
        "date": 1777456603327,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 72.7,
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
          "id": "847970a791d9ccde2a8d62467763289fc672771b",
          "message": "Merge pull request #83 from LoveDaisy/feat/globe_view\n\nfeat(gui): Globe lens with trackball, overlay labels, and view reset",
          "timestamp": "2026-04-30T23:23:42+08:00",
          "tree_id": "4c980cbd42b23b410fd148ee16eada28b2feccdb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/847970a791d9ccde2a8d62467763289fc672771b"
        },
        "date": 1777562898753,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 74.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 72.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 57.4,
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
          "id": "5c889e1e1f526d6bda40f029153821ff8a06fcf6",
          "message": "Merge pull request #84 from LoveDaisy/feat/ux_crystal_card\n\nfeat(gui): highlight active crystal card while edit modal is open",
          "timestamp": "2026-05-01T00:42:19+08:00",
          "tree_id": "1fe7d7881a3ce16e3cdfd44fdececc046f8f1afd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c889e1e1f526d6bda40f029153821ff8a06fcf6"
        },
        "date": 1777567620009,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 90.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.8,
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
          "id": "efafa43354b35f824c02394d62dbc328464441ba",
          "message": "Merge pull request #85 from LoveDaisy/feat/more_filters\n\nfeat: per-type filter subpanels (Raypath / EntryExit / Direction / Crystal) with .lmc v2 serialization",
          "timestamp": "2026-05-06T11:32:12+08:00",
          "tree_id": "c65cfd36e780f84271a95de1b162d2f55d96cbbf",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/efafa43354b35f824c02394d62dbc328464441ba"
        },
        "date": 1778038623147,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 76.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74.9,
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
          "id": "f04e1ad447d4f4ccdeef564955f7043422e706e6",
          "message": "Merge pull request #86 from LoveDaisy/feat/globe_view\n\nfeat: Globe↔other lens combo transform, visibility radio buttons, View panel regroup",
          "timestamp": "2026-05-06T12:27:33+08:00",
          "tree_id": "3291df96efef44c52eb9b8d5a8f0c2f7a1722641",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f04e1ad447d4f4ccdeef564955f7043422e706e6"
        },
        "date": 1778041926320,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 80.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 57.3,
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
          "id": "629b323b216916714ed81f72579bb132517b2088",
          "message": "Merge pull request #87 from LoveDaisy/feat/gui_polish\n\nfeat(gui-polish-v18): visibility row, drop Direction filter, EE align Raypath",
          "timestamp": "2026-05-07T12:18:21+08:00",
          "tree_id": "cba0b9455d12d07982d550091a56b3c28e98d010",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/629b323b216916714ed81f72579bb132517b2088"
        },
        "date": 1778127817754,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 78,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 70.8,
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
          "id": "702fec607e094345d83386ea8011b67d9ccc5718",
          "message": "Merge pull request #88 from LoveDaisy/feat/raypath_symmetry\n\nRaypath P/B/D symmetry redesign + fold/roll fix + e2e regression",
          "timestamp": "2026-05-09T12:44:27+08:00",
          "tree_id": "73a61c0e6a037827a94616b45044f837a5cd9b4f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/702fec607e094345d83386ea8011b67d9ccc5718"
        },
        "date": 1778302159332,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 96.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 71.3,
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
          "id": "940557be35656483ccb5c98a4312d48e7a6b8359",
          "message": "Merge pull request #89 from LoveDaisy/feat/auto_ev\n\nfeat: Adaptive Brightness (P99-anchored auto-EV) for GUI",
          "timestamp": "2026-05-13T09:57:49+08:00",
          "tree_id": "503baee7a3d600f6b16282f53e78474e03be1b8a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/940557be35656483ccb5c98a4312d48e7a6b8359"
        },
        "date": 1778637762990,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 72.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 67.5,
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
          "id": "94e4edf61c07a64640c38d90c1b2dc3fda974653",
          "message": "Merge pull request #90 from LoveDaisy/dev/refactor_c_api\n\nRefactor: enforce GUI/core API boundary via 4 new C API entries",
          "timestamp": "2026-05-14T20:00:37+08:00",
          "tree_id": "99603c8703022d6dc31ab9fa8440052434a0ba84",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/94e4edf61c07a64640c38d90c1b2dc3fda974653"
        },
        "date": 1778760321386,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 68.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 57.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 63.5,
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
          "id": "8d97704a2cc32149478a5e9fade9bfca321483d6",
          "message": "Merge pull request #91 from LoveDaisy/feat/update_test_ref\n\nUpdate GUI test references with mean-ref pipeline",
          "timestamp": "2026-05-14T20:18:50+08:00",
          "tree_id": "a2a8ea7ee23efaa4c5c619cec9b8b223fcaf5f5b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d97704a2cc32149478a5e9fade9bfca321483d6"
        },
        "date": 1778761403898,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 69.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 54.7,
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
          "id": "280bf0be7058816e808ba6a78cc98a7a5452eb6b",
          "message": "Merge pull request #92 from LoveDaisy/fix/raypath-1-3\n\nfix(core): repair raypath=[1,3] TIR edge leak via fid_in_src parameter",
          "timestamp": "2026-05-18T08:53:02+08:00",
          "tree_id": "8587ee20db4172496e5a7a3bd75b60244f2d8958",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/280bf0be7058816e808ba6a78cc98a7a5452eb6b"
        },
        "date": 1779065893997,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 59.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 64.3,
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
          "id": "fa8e6cfb354f00e5922cc8d458b330670ff7fb29",
          "message": "Merge pull request #93 from LoveDaisy/feat/query-filter-uplift-v2\n\nfeat(core): uplift query filter to RenderConsumer, fix OFF-mode normalization",
          "timestamp": "2026-05-18T10:04:37+08:00",
          "tree_id": "25b4ddbbd0c316a6ae1f3b02b2309815af457d9c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa8e6cfb354f00e5922cc8d458b330670ff7fb29"
        },
        "date": 1779070176069,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 65.1,
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
          "id": "b30b552674d681adb8b6123bcbea5954b3340e2f",
          "message": "Merge pull request #94 from LoveDaisy/feat/partition-additivity-test-redesign\n\nRedesign test_partition_buffer_additivity with discriminating metric",
          "timestamp": "2026-05-18T14:17:11+08:00",
          "tree_id": "8c193a6d69f9da7200a7ef970dd3842024d97fb2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b30b552674d681adb8b6123bcbea5954b3340e2f"
        },
        "date": 1779085347431,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 67.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 72.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 65.1,
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
          "id": "de1ce7ae90f4d4404c0fc55b92f4a267c52da934",
          "message": "Merge pull request #95 from LoveDaisy/feat/mesh_per_face\n\nfeat(core): scrum-193 leftovers batch — per-face topology + polygon-only tracing",
          "timestamp": "2026-05-18T18:12:13+08:00",
          "tree_id": "e40e53678e278c0e378b085931d4091084ba7f98",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de1ce7ae90f4d4404c0fc55b92f4a267c52da934"
        },
        "date": 1779099458375,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 101.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 69.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 63.5,
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
          "id": "5908ce3f2446e604b823aeea72293bb87451f589",
          "message": "Merge pull request #96 from LoveDaisy/dev/refactor_rayseg\n\nrefactor: redesign RaySeg state + sentinel (scrum-206)",
          "timestamp": "2026-05-19T09:32:23+08:00",
          "tree_id": "f209b2da889e6a16d9d82f574c0cc0695d8140f3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5908ce3f2446e604b823aeea72293bb87451f589"
        },
        "date": 1779154654128,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 69.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 64.6,
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
          "id": "c680dcc6a69433924731db037eb416920fd012a4",
          "message": "Merge pull request #97 from LoveDaisy/feat/editor_window_ux\n\nfeat(gui): edit modal UX polish (raypath buffer + pyramid slider alignment)",
          "timestamp": "2026-05-19T11:22:18+08:00",
          "tree_id": "b896575144a36ab9ccd7b0d844d85b5edf0a7929",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c680dcc6a69433924731db037eb416920fd012a4"
        },
        "date": 1779161221173,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 98.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 73.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 66.6,
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
          "id": "35e35f3275c09d626a2ebadc49c026d8d790ff17",
          "message": "Merge pull request #98 from LoveDaisy/feat/filter_performance\n\nfeat(core): canonical-form FilterSpec + fix per-ray symmetry hot path",
          "timestamp": "2026-05-20T09:41:53+08:00",
          "tree_id": "fb556e78e23a0fafa29a2be152e168d43f33bfa3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35e35f3275c09d626a2ebadc49c026d8d790ff17"
        },
        "date": 1779241581911,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 89.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 70.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 66.1,
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
          "id": "ea5ab5ed603b0df052058857dc75ee0a822d6df4",
          "message": "Merge pull request #99 from LoveDaisy/feat/ux_polish\n\nfeat(gui): ux polish v2 + dead-code cleanup (panels.cpp::SliderWithPreset)",
          "timestamp": "2026-05-20T11:55:13+08:00",
          "tree_id": "286ea92e04c4c205a715bcc8d8867c7166e0bcdd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ea5ab5ed603b0df052058857dc75ee0a822d6df4"
        },
        "date": 1779249587884,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 75.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 63.9,
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
          "id": "2506276291a8c48473a44af1f0e849671d260ff4",
          "message": "Merge pull request #100 from LoveDaisy/fix/capi\n\nfix(c-api): guard sentinel write past max_count + slow regression test",
          "timestamp": "2026-05-20T16:47:27+08:00",
          "tree_id": "64f28a12c3e3144dd31b56a51b4ab9f90d1e4a96",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2506276291a8c48473a44af1f0e849671d260ff4"
        },
        "date": 1779267140077,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 65.7,
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
          "id": "dec91f357ae70588ef872acc48f9e3268e0d21aa",
          "message": "Merge pull request #101 from LoveDaisy/fix/capi_shared_lib\n\nfix(gui-shared-lib): expose XyzToSrgbUint8 via C API; restore GUI/core boundary",
          "timestamp": "2026-05-20T18:14:57+08:00",
          "tree_id": "5f0da1e0c5f0e2ce6ff584f3de125f66f308436c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dec91f357ae70588ef872acc48f9e3268e0d21aa"
        },
        "date": 1779272422909,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 111,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 67.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 68.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 64.7,
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
          "id": "135151eed06f1cb0288a1020d0a10c52cc45c485",
          "message": "Merge pull request #102 from LoveDaisy/feat/gui_font\n\nfeat(gui): integrate FontAwesome 6 icon font for 12 GUI buttons",
          "timestamp": "2026-05-21T12:50:01+08:00",
          "tree_id": "949a6505c17485e7d2383f3d8f4f9f6d89603285",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/135151eed06f1cb0288a1020d0a10c52cc45c485"
        },
        "date": 1779339327320,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 73.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 66.2,
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
          "id": "d158a9394600b1ded192e6dc974603f42b76fb47",
          "message": "Merge pull request #103 from LoveDaisy/feat/gui_card_link\n\nfeat(gui): linked entries via ID-pool model + pick-mode UX",
          "timestamp": "2026-05-23T17:41:59+08:00",
          "tree_id": "dcb404af64a0c4302dbf917a45e0f4888b230422",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d158a9394600b1ded192e6dc974603f42b76fb47"
        },
        "date": 1779529616960,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 89.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 73.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 67.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 65.5,
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
          "id": "dd136e1208d0ad1d81384632592cc2c9421597ab",
          "message": "Merge pull request #104 from LoveDaisy/fix/ms_prob_leak\n\nfix(filter): restore filter as simulator emit-gate (Design A) + architecture doc",
          "timestamp": "2026-05-23T17:55:00+08:00",
          "tree_id": "e624cd047ec6fa17909614f4f428aa2ee3d96f8f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dd136e1208d0ad1d81384632592cc2c9421597ab"
        },
        "date": 1779530386198,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 92.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 86.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 69.4,
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
          "id": "e925a47e0db8e111ca3bd397a8ed21c44f8ed48d",
          "message": "Merge pull request #105 from LoveDaisy/fix/adaptive_brightness_screenshot\n\nAdaptive Brightness: single-mode F1 anchor + screenshot fix + P99.5/T135 defaults",
          "timestamp": "2026-05-24T20:37:24+08:00",
          "tree_id": "a7d961fcc9ac82180aab92bb7165fd154a4c4616",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e925a47e0db8e111ca3bd397a8ed21c44f8ed48d"
        },
        "date": 1779626511695,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 80.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 72.2,
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
          "id": "fc3e2fa37ad9f4bce408a9a5817f09eb57aa7c8a",
          "message": "Merge pull request #106 from LoveDaisy/feat/gui_zenith_nadir\n\nfeat(gui): zenith/nadir ring marker overlay",
          "timestamp": "2026-05-25T01:23:44+08:00",
          "tree_id": "3f5880b3f64c960c7f36c1b0c568f0b38f820075",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc3e2fa37ad9f4bce408a9a5817f09eb57aa7c8a"
        },
        "date": 1779643720518,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 79.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 75.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 72.9,
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
          "id": "22dc93834aba6b5b8c0e55d16027165f830004f8",
          "message": "Merge pull request #107 from LoveDaisy/feat/ci_test\n\nci: add slow e2e test job with shared-lib build",
          "timestamp": "2026-05-25T09:25:31+08:00",
          "tree_id": "8bdc09a9c9a6566dab36fe1e61008d26ef894aab",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/22dc93834aba6b5b8c0e55d16027165f830004f8"
        },
        "date": 1779672710643,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 90.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 79.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 76,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 71.6,
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
          "id": "e6da1a081780ca1412d671fc3108615df715b988",
          "message": "Merge pull request #108 from LoveDaisy/fix/ms_filter_leak\n\nfix(simulator): prevent MS filter-leaked rays from reaching main output",
          "timestamp": "2026-05-25T14:17:56+08:00",
          "tree_id": "f0ddc6dd281209f960a42d9ba62a62b1e4389732",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e6da1a081780ca1412d671fc3108615df715b988"
        },
        "date": 1779690122741,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 93.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 84.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 71.7,
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
          "id": "b555bce0cfb41ac2959a09682690abaee59e4dcc",
          "message": "Merge pull request #109 from LoveDaisy/dev/env_var\n\nrefactor: replace LUMICE_SIM_SEED env var with C API sim_seed parameter",
          "timestamp": "2026-05-25T15:12:52+08:00",
          "tree_id": "7b0e4e2ac9d8d9d06a5c565821e5ba98db920649",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b555bce0cfb41ac2959a09682690abaee59e4dcc"
        },
        "date": 1779693466698,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 91.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 80.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 60.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 69.1,
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
          "id": "57735b3bf85ff7ae78b102ea0d93aca76e34cc73",
          "message": "Merge pull request #110 from LoveDaisy/feat/render_refresh\n\nFix rendering update issues, simulator audit gaps, and raise max hits",
          "timestamp": "2026-05-26T01:05:00+08:00",
          "tree_id": "9b9aab40e5072b0b7c0942bdb8ed63184483a299",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/57735b3bf85ff7ae78b102ea0d93aca76e34cc73"
        },
        "date": 1779728988824,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 80.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 72,
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
          "id": "9db583df928264e5fe43a90e364da883ed197704",
          "message": "Merge pull request #111 from LoveDaisy/feat/gui_minor\n\nfeat(gui): card-wide click and front-hemisphere checkbox",
          "timestamp": "2026-05-27T08:18:34+08:00",
          "tree_id": "3010b3aeccd240cd41893b564209d19ca5731165",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9db583df928264e5fe43a90e364da883ed197704"
        },
        "date": 1779841361471,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 92.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 71.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 71.1,
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
          "id": "062fff11d9ca5a48e9424fc2424572c415b9460f",
          "message": "Merge pull request #112 from LoveDaisy/fix/misc\n\nchore(gui): unify overlay sentinel and add review follow-up fixes",
          "timestamp": "2026-05-27T08:42:42+08:00",
          "tree_id": "adcb4da74d7fbf0987da6ef3c60a107b7c34e64a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/062fff11d9ca5a48e9424fc2424572c415b9460f"
        },
        "date": 1779842848804,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 79.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 74.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 60.1,
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
          "id": "eb70214863932c839b9f6102a4ce5961856333bc",
          "message": "Merge pull request #113 from LoveDaisy/chore/gui_lag\n\nfix(gui): eliminate high-resolution GUI lag",
          "timestamp": "2026-05-27T10:08:15+08:00",
          "tree_id": "a7433ce2d4a833d2ea80f18d110d7ea3ce15f183",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/eb70214863932c839b9f6102a4ce5961856333bc"
        },
        "date": 1779847936908,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 83.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 64.7,
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
          "id": "e4a3f6587b8ef382e420de0b5d739f4f5bc0d8c5",
          "message": "Merge pull request #114 from LoveDaisy/chore/major_audit\n\nchore: major audit — architecture docs, tooltips, and cross-ref comments",
          "timestamp": "2026-05-28T08:33:03+08:00",
          "tree_id": "aabe60b8c5d31e9c2a26d6fc057ffc6fbfe12dad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e4a3f6587b8ef382e420de0b5d739f4f5bc0d8c5"
        },
        "date": 1779928710991,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 79.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 59.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 64.9,
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
          "id": "e3d86034ece73c9d5b32a9981fc5b324bdbfbce6",
          "message": "Merge pull request #115 from LoveDaisy/feat/remove_ev_anchor\n\nRemove F1 anchor lane, use self-P99.5 EV normalization",
          "timestamp": "2026-05-28T15:38:16+08:00",
          "tree_id": "52693d70e631b7a71599cdcad16ec33dd9f6c0d1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e3d86034ece73c9d5b32a9981fc5b324bdbfbce6"
        },
        "date": 1779954200315,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 91.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 82.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 67.2,
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
          "id": "dc2918aa9d575eeb2b3007788b1bb8cd62f29193",
          "message": "Merge pull request #116 from LoveDaisy/feat/ev-anchor-p99\n\nfeat(auto-ev): rework adaptive brightness anchor (P99 + global f=8 downsample metric)",
          "timestamp": "2026-05-31T20:02:04+08:00",
          "tree_id": "cd1482b671d823bc75c304687bebc49aaa388c11",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dc2918aa9d575eeb2b3007788b1bb8cd62f29193"
        },
        "date": 1780229220249,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 77.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 80.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 71.7,
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
          "id": "dfad7e8b454dfdd7903349cf553465cdcacf91e9",
          "message": "Merge pull request #117 from LoveDaisy/chore/ev-pipeline-doc-sync\n\ndocs: align EV/filter architecture docs to single-lane (post anchor-lane removal)",
          "timestamp": "2026-06-01T00:01:46+08:00",
          "tree_id": "bcc818e5077e221eaa5599a098d72d4c7e60f479",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/dfad7e8b454dfdd7903349cf553465cdcacf91e9"
        },
        "date": 1780243629587,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 90.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 78.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 76.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 69.8,
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
          "id": "d726d4f8d498793da4007ea5f1b139f12c7d5a9d",
          "message": "Merge pull request #118 from LoveDaisy/feat/ee-filter-uplift\n\nfeat: EntryExit filter expressiveness — wildcard, multi-value OR, length bounds",
          "timestamp": "2026-06-01T12:57:04+08:00",
          "tree_id": "2d104ee146202faca56b9fd9ed006a45cf215c55",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d726d4f8d498793da4007ea5f1b139f12c7d5a9d"
        },
        "date": 1780290115285,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 79.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.5,
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
          "id": "a83345b608ebc03c14a674ba2d9e86a766c27e61",
          "message": "Merge pull request #119 from LoveDaisy/dev/soa_refactor\n\nCPU SoA: ray recorder footprint optimization (RaypathRecorder split + SBO)",
          "timestamp": "2026-06-02T14:44:34+08:00",
          "tree_id": "54bf6808db99db93563899e56f190d727a9b2868",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a83345b608ebc03c14a674ba2d9e86a766c27e61"
        },
        "date": 1780382993976,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 74.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 72.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 52.7,
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
          "id": "392ee041c7255f883dcb83f6eb4c4958ea8f8d28",
          "message": "Merge pull request #120 from LoveDaisy/dev/perf_opt2\n\nperf(worker-default): default worker count to physical cores",
          "timestamp": "2026-06-03T18:05:54+08:00",
          "tree_id": "0863f2e30f743b4cf8bdb1004f290e0da8e43762",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/392ee041c7255f883dcb83f6eb4c4958ea8f8d28"
        },
        "date": 1780481472540,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 98.6,
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
          "id": "390813463050266f30c6406eab1cb4b9a52972bb",
          "message": "Merge pull request #121 from LoveDaisy/feat/metal_backend_prod\n\nfeat: Metal trace backend (production) — pluggable TraceBackend seam + CPU/Metal parity",
          "timestamp": "2026-06-08T17:19:20+08:00",
          "tree_id": "ce38b259f1a458387effaafcbece2fdfa5c2e196",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/390813463050266f30c6406eab1cb4b9a52972bb"
        },
        "date": 1780910681665,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 97.3,
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
          "id": "5c588072df470acedf8b0eb419f2c1b278e77239",
          "message": "Merge pull request #122 from LoveDaisy/feat/metal-gui-default\n\nfeat: enable Metal backend on GUI live-preview (dual-fisheye-EA + C API toggle)",
          "timestamp": "2026-06-09T09:33:36+08:00",
          "tree_id": "d8c695861215eb9ce11c1bf5c993c7a78a99cece",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c588072df470acedf8b0eb419f2c1b278e77239"
        },
        "date": 1780969101182,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 91.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 99.5,
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
          "id": "ca327679e29ff539e1ab7a1b51177ed6066af02f",
          "message": "Merge pull request #123 from LoveDaisy/feat/metal-exit-seam\n\nfeat(metal): exit-seam buffer egress 出口替代整幅回读 (P1/scrum-258)",
          "timestamp": "2026-06-12T09:45:24+08:00",
          "tree_id": "0af735a07c022e2c17115e13424724ac9d00cca0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ca327679e29ff539e1ab7a1b51177ed6066af02f"
        },
        "date": 1781229036649,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 92.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 104.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 97,
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
          "id": "c5b5985c34ddd97e7372b393a1d7892279bf136c",
          "message": "Merge pull request #124 from LoveDaisy/feat/metal-rootgen\n\nfeat(metal): device GPU root-gen 默认根供给 + 吞吐确证 (P2/scrum-260+261)",
          "timestamp": "2026-06-12T10:11:44+08:00",
          "tree_id": "7e1566edab98bcbd5054d82dd2e28bbc19b99c7e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c5b5985c34ddd97e7372b393a1d7892279bf136c"
        },
        "date": 1781230756249,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 104,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 100,
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
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "207de068d30e8080608200a0b1cf9b54c3518f43",
          "message": "Merge pull request #125 from LoveDaisy/fix/serverimpl-stop-deadlock\n\nfix: ServerImpl::Stop() condition-variable lost-wakeup deadlock + stress regression",
          "timestamp": "2026-06-12T11:53:36+08:00",
          "tree_id": "476d50f4d2d83297176fb1ddb991155d55897e02",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/207de068d30e8080608200a0b1cf9b54c3518f43"
        },
        "date": 1781236765808,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.8,
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
          "id": "00f9d49fc95fe411dd82ab6686e21fc0de5718e2",
          "message": "Merge pull request #126 from LoveDaisy/feat/metal-gen-trace-fusion\n\nfeat(metal): fuse gen+trace into one command buffer (task-264)",
          "timestamp": "2026-06-13T01:45:22+08:00",
          "tree_id": "9aed83822638731d531d0ae74a7dc2df39d40d92",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/00f9d49fc95fe411dd82ab6686e21fc0de5718e2"
        },
        "date": 1781286657434,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 97.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 99,
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
          "id": "f007991d19cb9c206a4966a0d19133f8838a6762",
          "message": "Merge pull request #127 from LoveDaisy/docs/gpu-seam-design-and-knowledge-base\n\ndocs: promote GPU seam-design blueprint + route history, add knowledge-base discipline",
          "timestamp": "2026-06-13T11:32:14+08:00",
          "tree_id": "76d13ea6ab4d6dc4c0579717acb972e4c3508ee9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f007991d19cb9c206a4966a0d19133f8838a6762"
        },
        "date": 1781321889136,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 97.9,
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
          "id": "064c866e25968157761e3751c93fadd158b56ffd",
          "message": "Merge pull request #128 from LoveDaisy/feat/gpu-single-engine-impl01\n\nfeat(gpu): device-resident continuation engine (§5 Scrum 1)",
          "timestamp": "2026-06-15T09:05:25+08:00",
          "tree_id": "002c927b3387b6839b4d22e45715f46c43bb28f6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/064c866e25968157761e3751c93fadd158b56ffd"
        },
        "date": 1781485863648,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 92.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.5,
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
          "id": "66f00330c4afa19932534d2a4f993d45d4fd086e",
          "message": "Merge pull request #129 from LoveDaisy/feat/gpu-single-engine-orchestration\n\nfeat(scrum-268): GPU single-engine orchestration — Metal beats legacy in the GUI",
          "timestamp": "2026-06-17T01:11:26+08:00",
          "tree_id": "b8dfda8d13194e60545a1f9f15103718a799d237",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/66f00330c4afa19932534d2a4f993d45d4fd086e"
        },
        "date": 1781630251372,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.6,
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
          "id": "5d0ae08f714a1ffe04c84be055d268ac1a7cf78b",
          "message": "Merge pull request #130 from LoveDaisy/chore/milestone-cleanup\n\nmilestone-cleanup: testing architecture + suite reorg + boundary/doc hardening (scrum-270)",
          "timestamp": "2026-06-17T17:47:17+08:00",
          "tree_id": "5732ebb8dd203a1d9d03c74eff462eadeab90348",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5d0ae08f714a1ffe04c84be055d268ac1a7cf78b"
        },
        "date": 1781690103361,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 113.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 104.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.7,
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
          "id": "44a9f65ea41f3d27b6a3312eaa0400e155dec8a4",
          "message": "Merge pull request #131 from LoveDaisy/perf/metal-gui-optimization\n\nperf: throughput measurement honesty + benchmark scene registry",
          "timestamp": "2026-06-19T15:01:39+08:00",
          "tree_id": "3ad22be58f5edb9734fe8e2cc078a723777417ca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/44a9f65ea41f3d27b6a3312eaa0400e155dec8a4"
        },
        "date": 1781852875808,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 66.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 91.6,
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
          "id": "51e34348e18bd1fa90c79792dd33c0adf8f4e3ba",
          "message": "Merge pull request #133 from LoveDaisy/fix/geometry-gen-numerical-robustness\n\nfix: extreme-wedge geometry-gen — kill fake basal face (B-ring raypath/subsun anomalies)",
          "timestamp": "2026-06-20T02:19:25+08:00",
          "tree_id": "da51e96d4de912b9c74faa6513b39f7c72ec7720",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/51e34348e18bd1fa90c79792dd33c0adf8f4e3ba"
        },
        "date": 1781893509377,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 69.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 89.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49699333+dependabot[bot]@users.noreply.github.com",
            "name": "dependabot[bot]",
            "username": "dependabot[bot]"
          },
          "committer": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "distinct": true,
          "id": "17544d966024fb8652dc9119b52ec05a8644c2fc",
          "message": "chore(deps): bump actions/checkout from 6 to 7\n\nBumps [actions/checkout](https://github.com/actions/checkout) from 6 to 7.\n- [Release notes](https://github.com/actions/checkout/releases)\n- [Changelog](https://github.com/actions/checkout/blob/main/CHANGELOG.md)\n- [Commits](https://github.com/actions/checkout/compare/v6...v7)\n\n---\nupdated-dependencies:\n- dependency-name: actions/checkout\n  dependency-version: '7'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-06-20T08:36:44+08:00",
          "tree_id": "80bfef8b5c3cab32e14340e0a36015d51a662732",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/17544d966024fb8652dc9119b52ec05a8644c2fc"
        },
        "date": 1781916145517,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 66.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 85.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
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
          "id": "242e72c6a0abf8e05eb6ac8e298539c4e5308fc6",
          "message": "Merge pull request #135 from LoveDaisy/fix/polygon-face-of-tri-argmax\n\nfix(core): PolygonFaceOfTri argmax for extreme-wedge entry-face mapping",
          "timestamp": "2026-06-20T21:20:27+08:00",
          "tree_id": "cd3d054caf4903ec76fba4d5a0f0b044071670ce",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/242e72c6a0abf8e05eb6ac8e298539c4e5308fc6"
        },
        "date": 1781962001043,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 68.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 90.9,
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
          "id": "6241754934a5a559fea3faf320cca1adf7b32c55",
          "message": "Merge pull request #136 from LoveDaisy/chore/land-robustness-doc\n\ndocs: land numerical-robustness conventions doc + AGENTS index (chore 280.1)",
          "timestamp": "2026-06-20T22:57:44+08:00",
          "tree_id": "21a30aa319e31dad0cb42f69d60da88bf99f0708",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6241754934a5a559fea3faf320cca1adf7b32c55"
        },
        "date": 1781967802564,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 65.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 89.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 93.4,
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
          "id": "e89453be18b912a0a8d3c630f0c7e330ac2823bd",
          "message": "Merge pull request #137 from LoveDaisy/feat/geometry-numerical-hardening\n\nfix(core/gui): geometry numerical hardening (scrum-280, explore-279 follow-up)",
          "timestamp": "2026-06-21T03:03:02+08:00",
          "tree_id": "3474b4be5cd4c31bdf88716b143bfa112038ef66",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e89453be18b912a0a8d3c630f0c7e330ac2823bd"
        },
        "date": 1781982569848,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 67.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 88,
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
          "id": "e74ee69ccb36da092f220b1f29f65b376b4e23e2",
          "message": "Merge pull request #138 from LoveDaisy/feat/backend-availability-gui-gate\n\nfeat(gui): runtime backend-availability gate for Metal checkbox",
          "timestamp": "2026-06-21T06:50:12+08:00",
          "tree_id": "64b806e212752d1db31e147b0ce478a9c9c1d06d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e74ee69ccb36da092f220b1f29f65b376b4e23e2"
        },
        "date": 1781996187388,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 73.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 90.2,
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
          "id": "3afcb8103a166e60f969c08f771ce940e87e4834",
          "message": "Merge pull request #139 from LoveDaisy/fix/metal-backend-crash\n\nfix: Metal backend crash on macOS 26.5 — graceful degradation + build-time precompiled metallib",
          "timestamp": "2026-06-22T21:09:18+08:00",
          "tree_id": "c42bf22a1ac73742be973f3c822976d5c4335382",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3afcb8103a166e60f969c08f771ce940e87e4834"
        },
        "date": 1782134165764,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 87.5,
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
          "id": "855e6467599128d7249358a9e7fddd24a1ae9d9d",
          "message": "Merge pull request #140 from LoveDaisy/fix/max-hits-overflow-crash\n\nfix(task-284): max_hits>15 CPU crash (EmplaceBack arena dup) + no-truncation exit-seam",
          "timestamp": "2026-06-23T00:04:58+08:00",
          "tree_id": "e8c0acda0c5c32dd47294ac7226ee4de751f408d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/855e6467599128d7249358a9e7fddd24a1ae9d9d"
        },
        "date": 1782144663852,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 66.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 92.3,
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
          "id": "c814390955ab88969cd00bf92a90d7f81c3773d5",
          "message": "Merge pull request #141 from LoveDaisy/task/env-hardening\n\nEnv-var policy + hardening: centralize knobs, --backend CLI flag, executable gate",
          "timestamp": "2026-06-23T11:01:11+08:00",
          "tree_id": "fe17f04aaff9d0402650e096618af68e19f8e90f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c814390955ab88969cd00bf92a90d7f81c3773d5"
        },
        "date": 1782184064688,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 67.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 83.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.5,
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
          "id": "3e0e8e00c10b327af88d8f94b780a29afbdbf592",
          "message": "Merge pull request #142 from LoveDaisy/feat/overlay-refactor\n\nfeat(scrum-288): overlay marker-lines projection completeness + screen-space consistency",
          "timestamp": "2026-06-24T00:10:15+08:00",
          "tree_id": "bca38c2f8909ce5cddc1987abd1ef4f1317c232b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3e0e8e00c10b327af88d8f94b780a29afbdbf592"
        },
        "date": 1782231431660,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 60.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89.7,
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
          "id": "d61c0aaa76b395c0fe4aaf460569b961e3157bbc",
          "message": "Merge pull request #143 from LoveDaisy/chore/backlog-minor-cleanups\n\nchore: batch of small backlog cleanups (auto-ev refs, deferred review minors, macOS rpath)",
          "timestamp": "2026-06-24T08:55:00+08:00",
          "tree_id": "986d272dc281d1208afcfc7c292929aea99ffa4f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d61c0aaa76b395c0fe4aaf460569b961e3157bbc"
        },
        "date": 1782262849635,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 90.6,
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
          "id": "de095baa6b90e83b8311844f3afca2f828153e09",
          "message": "Merge pull request #144 from LoveDaisy/chore/cleanup-deferred-misc-batch\n\nchore: batch-clean 5 deferred misc backlog items (#292)",
          "timestamp": "2026-06-24T09:59:01+08:00",
          "tree_id": "2432a8f38c538bfa0e3c7455b05eade491dadf3d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de095baa6b90e83b8311844f3afca2f828153e09"
        },
        "date": 1782266703459,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 89.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 85.7,
            "unit": "%"
          }
        ]
      }
    ]
  }
}