window.BENCHMARK_DATA = {
  "lastUpdate": 1788435091042,
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
          "id": "6ee81825abca90286b1eaef923f6aeaaaabae056",
          "message": "Merge pull request #202 from LoveDaisy/feat/filter-form-big-or\n\nfeat: 放开 filter OR-clause 上限 8/16→4096(纯过滤,染色 mask 不动)",
          "timestamp": "2026-07-16T07:43:52+08:00",
          "tree_id": "c02e2eebc9625ec984d3d85fbb779cf8f25ea060",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6ee81825abca90286b1eaef923f6aeaaaabae056"
        },
        "date": 1784159457397,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 437881.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584634.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 384094.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 373585.8,
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
          "id": "ec1879077f613c9bbfaabb3079281ef5164bb2de",
          "message": "Merge pull request #203 from LoveDaisy/chore/reconciler-gate-wake-helper\n\nchore: harden reconciler include boundary + dedup wake path (scrum-353 T2 follow-up)",
          "timestamp": "2026-07-16T08:44:05+08:00",
          "tree_id": "b12a3a7a06d99002a3d740ef5c655ebef8581689",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ec1879077f613c9bbfaabb3079281ef5164bb2de"
        },
        "date": 1784163001676,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 362318.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588887.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 385555.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 456617.4,
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
          "id": "c89c178f203a9956ee50c8af21c7f0300615d053",
          "message": "Merge pull request #204 from LoveDaisy/feat/color-degrade-gui-surfacing\n\nfeat(color-degrade-gui-surfacing): surface all 3 GPU color-degrade caps to GUI modal",
          "timestamp": "2026-07-16T11:53:31+08:00",
          "tree_id": "ac0cf27c253b945d4191e4e914f50f1ad145a0fe",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c89c178f203a9956ee50c8af21c7f0300615d053"
        },
        "date": 1784174447348,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 465776.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 598866.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 616815.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 351447.5,
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
          "id": "ab7aa0b509b61ae45afbf6b3f6fb7846507c53b1",
          "message": "Merge pull request #205 from LoveDaisy/chore/policy-gates\n\nchore(policy-gates): fix the bench compile rot and gate working-note references",
          "timestamp": "2026-07-17T07:52:34+08:00",
          "tree_id": "c40692428310e7ecfc1ae1acafbfeb140902be36",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ab7aa0b509b61ae45afbf6b3f6fb7846507c53b1"
        },
        "date": 1784246395144,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 467098.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 590823.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 410336.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 351901.8,
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
          "id": "a610dbde93b0957805c532933a5bde34fabc21a2",
          "message": "Merge pull request #206 from LoveDaisy/fix/degenerate-geometry\n\nfix(core): random face_distance SIGSEGV — scale-relative vertex dedup + non-manifold rejection",
          "timestamp": "2026-07-17T15:08:47+08:00",
          "tree_id": "771c0d3b33012a43a90f8696bf92c5510957945e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a610dbde93b0957805c532933a5bde34fabc21a2"
        },
        "date": 1784272565261,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 455429.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586330.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 385583.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 349470.3,
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
          "id": "489bfb6288808f55579d34cc8361b00fd84d8fb0",
          "message": "Merge pull request #207 from LoveDaisy/docs/geom-clock-and-benchmark-caveats\n\ndocs: correct two measurement caveats found while calibrating the geometry clock",
          "timestamp": "2026-07-17T18:00:16+08:00",
          "tree_id": "56bd0351ee4a8d7916c831f46b0b8cc6b5ce0f59",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/489bfb6288808f55579d34cc8361b00fd84d8fb0"
        },
        "date": 1784282846812,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 442577.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 595314.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 383600.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 389064.6,
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
          "id": "e22f9eea8e5d88a52b44452e88488b73c5bc9ec0",
          "message": "Merge pull request #208 from LoveDaisy/fix/pyramid-geometry-crash-metal\n\nfix(core): pyramid + random face_distance Metal SIGSEGV (count/stride decouple)",
          "timestamp": "2026-07-18T12:55:41+08:00",
          "tree_id": "4664a91a1f187e7cb56fae546dfaa87d149d926c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e22f9eea8e5d88a52b44452e88488b73c5bc9ec0"
        },
        "date": 1784350986600,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 594036,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 496573.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 357024.9,
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
          "id": "f29881b3f101b6c06279e10954d789f5984d27bb",
          "message": "Merge pull request #210 from LoveDaisy/chore/gbk-locale-parity-test-unicode\n\nfix(test): ASCII-ize parity test messages for GBK-locale Windows",
          "timestamp": "2026-07-19T09:17:00+08:00",
          "tree_id": "ec1b7089cc6b1d84ac772796a218885ef9dfbe58",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f29881b3f101b6c06279e10954d789f5984d27bb"
        },
        "date": 1784424294588,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 461833.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585593.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400767.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 352880.8,
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
          "id": "abbef1227936e40b1753817142344886fae0de78",
          "message": "Merge pull request #209 from LoveDaisy/fix/cuda-unfreeze-geometry-randomization\n\nfix(cuda): unfreeze crystal-shape geometry randomization end-to-end",
          "timestamp": "2026-07-19T09:16:57+08:00",
          "tree_id": "836469c0bb3baf78eec16a14d2eb5e42760849c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/abbef1227936e40b1753817142344886fae0de78"
        },
        "date": 1784425807468,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 481881.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 595432.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 393219.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 356458.5,
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
          "id": "d9d7ee2ff6accd887235198435abfedf87e4f1b2",
          "message": "Merge pull request #212 from LoveDaisy/chore/fix-base\n\nChore/fix base",
          "timestamp": "2026-07-19T10:15:27+08:00",
          "tree_id": "63532f0f8cbe507abecaa438c81d729aee904549",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d9d7ee2ff6accd887235198435abfedf87e4f1b2"
        },
        "date": 1784427798045,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 478125.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594237.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 401526.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 353524.3,
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
          "id": "163ff642d86a80c43b8197a2ba57a5016cd6d6a7",
          "message": "Merge pull request #213 from LoveDaisy/feat/geometry-pool-and-topology-reuse\n\nfeat(geometry-perf): per-ray K-shape pool on both GPU backends + geometry representation diagnosis",
          "timestamp": "2026-07-20T11:37:25+08:00",
          "tree_id": "099175e5599952910c54ce6d8ff31717f756c00f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/163ff642d86a80c43b8197a2ba57a5016cd6d6a7"
        },
        "date": 1784519093773,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 398077.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592765.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400748.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 309762.5,
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
          "id": "4057ce13d077863ae1027d799f1fe8d4c8fe1cb0",
          "message": "Merge pull request #214 from LoveDaisy/feat/geometry-closed-form-representation\n\nfeat(geometry): closed-form hex crystal representation (scrum-386)",
          "timestamp": "2026-07-21T13:25:10+08:00",
          "tree_id": "03e5310167743bc37a0d3c76b1b4600e8b4983c0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4057ce13d077863ae1027d799f1fe8d4c8fe1cb0"
        },
        "date": 1784611969554,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 440453.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 599622.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400868.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 344058.3,
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
          "id": "0244b5f169af36f52d1fbfcf83ea3ca4df809f12",
          "message": "Merge pull request #215 from LoveDaisy/feat/geometry-exact-domain-audit\n\ngeometry exactness: symbolic-a1 exact oracle (drop __int128) + pyramid apex bug fix + 3-platform verify",
          "timestamp": "2026-07-22T15:28:14+08:00",
          "tree_id": "455c7b6b9c07b5fffdb7b5cc4cd6c4c0844c0eb7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/0244b5f169af36f52d1fbfcf83ea3ca4df809f12"
        },
        "date": 1784705723820,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 406651.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 601038.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400406.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321755.6,
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
          "id": "7bd2f401246d724694c7c02bbce7a37b093f43d4",
          "message": "Merge pull request #216 from LoveDaisy/feat/pyramid-oracle-contract-tests\n\ngeometry test: retire symbolic-α pyramid oracle for three contract-aligned tests",
          "timestamp": "2026-07-22T17:35:30+08:00",
          "tree_id": "316a03a93602e9c23a00832d413abf5c51648733",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7bd2f401246d724694c7c02bbce7a37b093f43d4"
        },
        "date": 1784713392958,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 470875.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593023.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 385521.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 338826.5,
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
          "id": "de7b4571c336a6d6f4814657c8822c6f149fb091",
          "message": "Merge pull request #217 from LoveDaisy/feat/geom-pool-metal-landing\n\nfeat(geom-pool): wire K-shape pool geom_clock into config + Metal/CUDA backends",
          "timestamp": "2026-07-22T23:04:47+08:00",
          "tree_id": "69bd58055ec2bb19e867aaf6a9ac66a07a16cb55",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de7b4571c336a6d6f4814657c8822c6f149fb091"
        },
        "date": 1784733120330,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 388508.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 598870.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 497944.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 326852.1,
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
          "id": "c527f60f46b67a4c4bb8bab34f73281efb072b65",
          "message": "Merge pull request #218 from LoveDaisy/feat/cuda-degenerate-geometry-parity\n\nfeat(cuda): degenerate K-shape pool parity + crystal-count assertion (scrum-392)",
          "timestamp": "2026-07-23T12:17:55+08:00",
          "tree_id": "0127101471b6d7dd59e5faea4bc8329d73e60df3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c527f60f46b67a4c4bb8bab34f73281efb072b65"
        },
        "date": 1784780740128,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 380493.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 603738,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 395235.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 343741.8,
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
          "id": "86b954890e231d91593eefe4a83ec335fe440227",
          "message": "Merge pull request #219 from LoveDaisy/feat/crystal-consumption-detriangulation\n\nfeat(core): detriangulate crystal consumption — polygon-granularity incidence sampling, remove triangle mesh from hot path",
          "timestamp": "2026-07-24T08:25:14+08:00",
          "tree_id": "7b83d1927fad720bef607c05a2de71ac1331ef3a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86b954890e231d91593eefe4a83ec335fe440227"
        },
        "date": 1784853150158,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 496000.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 579029.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 378296.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 315755.3,
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
          "id": "1459b4b094c37b2dd13657b05447d4a342cb6d1b",
          "message": "Merge pull request #220 from LoveDaisy/feat/strong-randomization-downstream-contracts\n\nfix: strong-randomization downstream contracts (filter/render/consumer)",
          "timestamp": "2026-07-24T18:21:13+08:00",
          "tree_id": "418f71e6a27d0ed077d6af076ab0f1ef0f95ab01",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1459b4b094c37b2dd13657b05447d4a342cb6d1b"
        },
        "date": 1784888921360,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 348970,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 581148.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398130.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 307345.7,
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
          "id": "85e12c4e4ae37fe7ffb660b9e22cce0eb96f71c3",
          "message": "Merge pull request #221 from LoveDaisy/feat/core-distribution-cleanup\n\nfeat: crystal shape randomization in GUI + first-class LUMICE_Distribution (BREAKING v4.10)",
          "timestamp": "2026-07-24T20:13:16+08:00",
          "tree_id": "32f4fc909d8d071013c9f3cfd519a135f6cfca2d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85e12c4e4ae37fe7ffb660b9e22cce0eb96f71c3"
        },
        "date": 1784895674237,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 280771.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592284,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398080.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 366617,
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
          "id": "fa613fb092061df691f456e4ee68497590efe136",
          "message": "Merge pull request #223 from LoveDaisy/feat/gui-shape-randomization-property-table\n\nfeat(gui): crystal shape randomization as a single uniform property table",
          "timestamp": "2026-07-25T09:15:06+08:00",
          "tree_id": "e97db9aed21e5466cd569d61f8ad02416099e8ee",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa613fb092061df691f456e4ee68497590efe136"
        },
        "date": 1784942598682,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 301139.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587298.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 391776.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345862.9,
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
          "id": "86adcd8f5fc57d211e201db64fb8d77f4bf37f88",
          "message": "Merge pull request #224 from LoveDaisy/feat/capi-scene-opaque-handle\n\nrefactor(c_api): LUMICE_Config value struct → LUMICE_Scene opaque handle (BREAKING v4.12)",
          "timestamp": "2026-07-25T21:38:42+08:00",
          "tree_id": "c1c944a06dc5cd04ab420b22aae79f66f9a92ced",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86adcd8f5fc57d211e201db64fb8d77f4bf37f88"
        },
        "date": 1784987158661,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 453743,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586464.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 377483.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 392122.2,
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
          "id": "62ba3d6ba31990e43588d1b8c222716cc1a70d5d",
          "message": "Merge pull request #225 from LoveDaisy/feat/gui-visual-regression-coverage\n\ntest(gui): reference-image pixel regression for lens projections + modal layouts",
          "timestamp": "2026-07-25T22:04:33+08:00",
          "tree_id": "750dc3e4cce3891a10ee627c0a5fde4efbbc3a21",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/62ba3d6ba31990e43588d1b8c222716cc1a70d5d"
        },
        "date": 1784988675385,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 467091.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 581161.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 378439,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 424054.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
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
          "id": "55bfc00af291844e9883a3a68e4c7ac4bc9a8bb3",
          "message": "chore(deps): bump actions/setup-python from 6 to 7\n\nBumps [actions/setup-python](https://github.com/actions/setup-python) from 6 to 7.\n- [Release notes](https://github.com/actions/setup-python/releases)\n- [Commits](https://github.com/actions/setup-python/compare/v6...v7)\n\n---\nupdated-dependencies:\n- dependency-name: actions/setup-python\n  dependency-version: '7'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-07-25T22:18:22+08:00",
          "tree_id": "f42b07d44e062a9971d1edbbc7b7be647d058b05",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/55bfc00af291844e9883a3a68e4c7ac4bc9a8bb3"
        },
        "date": 1784989583965,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 449980.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585169.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 386513.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 346492.2,
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
          "id": "28e5a9de538df35ed7b5b980592cd051c1109e71",
          "message": "Merge pull request #226 from LoveDaisy/chore/dead-weight-closeout\n\nchore(dead-weight-closeout): delete orphaned Server::CommitConfigFromFile, fix stale ExitRayRecord size comments",
          "timestamp": "2026-07-25T23:16:52+08:00",
          "tree_id": "6622931ec23b0c2436e0728a5bdcc31f2e48b31d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/28e5a9de538df35ed7b5b980592cd051c1109e71"
        },
        "date": 1784993011298,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 467793.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584472.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 399365.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 385803.1,
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
          "id": "b8248749346e73a9c28c6d9b8c8e68d688f477bc",
          "message": "Merge pull request #227 from LoveDaisy/chore/unified-logging-gate\n\nchore: route src/ diagnostics through the logger, gate bare prints",
          "timestamp": "2026-07-26T00:33:32+08:00",
          "tree_id": "778d66a5d877fafaa592ec918b1102342dcec321",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b8248749346e73a9c28c6d9b8c8e68d688f477bc"
        },
        "date": 1784997684857,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 434454.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588118.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 467201.7,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 320123,
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
          "id": "9b530515528d848ef297c39f68be7d2b0db36841",
          "message": "Merge pull request #228 from LoveDaisy/feat/face-distance-sync-groups\n\nfeat: shape-scalar sync groups (symmetry-preserving shape randomization)",
          "timestamp": "2026-07-27T14:08:08+08:00",
          "tree_id": "e8c7e3774d16d984526ed00e175883c3166fbb79",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9b530515528d848ef297c39f68be7d2b0db36841"
        },
        "date": 1785133047148,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 326103.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586417.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 385936.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 349190.5,
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
          "id": "1104a952b08241bd61e45bcf7380439a761cbbf7",
          "message": "Merge pull request #229 from LoveDaisy/feat/crystal-sample-count-semantics\n\nfix(stats): make crystals=N a scene property instead of a schedule artifact",
          "timestamp": "2026-07-27T15:17:09+08:00",
          "tree_id": "330fc333a3c88c2f6fcf80ba8e69d53bbf64b031",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1104a952b08241bd61e45bcf7380439a761cbbf7"
        },
        "date": 1785137100278,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 388012.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 589003.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 390834.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 322308.2,
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
          "id": "ce07b6aea91996dafbf320883cdcd408d861b672",
          "message": "Merge pull request #230 from LoveDaisy/feat/shape-schema-key-single-source\n\nrefactor: give the crystal shape/axis JSON keys one owner in core",
          "timestamp": "2026-07-27T17:39:30+08:00",
          "tree_id": "e1b5c4d911db4ddbe74f30e0e534b9eed7be52b1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ce07b6aea91996dafbf320883cdcd408d861b672"
        },
        "date": 1785145666368,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 463509.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586480.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 495698.7,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 356315.1,
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
          "id": "19ff683644d3f8e69afc7f9c3226be0b5971d9b1",
          "message": "Merge pull request #231 from LoveDaisy/feat/user-defaults\n\nfeat(gui): user-level defaults layer — generated diff panel + editable axis preset library",
          "timestamp": "2026-07-28T09:03:52+08:00",
          "tree_id": "705f62fb463dfc4fa6d35c7c57aa2735e68ff882",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/19ff683644d3f8e69afc7f9c3226be0b5971d9b1"
        },
        "date": 1785201078944,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 328296.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586369.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 381055.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 575737.4,
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
          "id": "a9d95c29cf1c41e15ecd018702019bef1d7f8f5d",
          "message": "Merge pull request #232 from LoveDaisy/feat/orientation-sample-count-stat\n\nfeat(stats): report orientation sample count as an independent statistic",
          "timestamp": "2026-07-28T13:40:08+08:00",
          "tree_id": "eac8e9336b7adffc48a245cc5cf3dc47ddc6764c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a9d95c29cf1c41e15ecd018702019bef1d7f8f5d"
        },
        "date": 1785217721112,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 315149.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582959.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 399586.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 331732.7,
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
          "id": "c8492857cbb930e8415baf8267aaadf2082f7b6e",
          "message": "Merge pull request #233 from LoveDaisy/feat/local-test-scope-and-docs\n\nbuild: per-flavor build/install trees + gate bare pytest to the fast subset",
          "timestamp": "2026-07-29T09:07:07+08:00",
          "tree_id": "7796a30d82f61ff9812359a3848ac7bbeb8ccc0a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c8492857cbb930e8415baf8267aaadf2082f7b6e"
        },
        "date": 1785287713696,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 417045.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 580098.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397310.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 317822.8,
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
          "id": "9955084a05d75819f74dff651c10a063ef2f5b65",
          "message": "Merge pull request #234 from LoveDaisy/feat/panel-settings-editor\n\nfeat(gui): make the defaults panel a pure editor with one source of constraint truth",
          "timestamp": "2026-07-29T09:36:30+08:00",
          "tree_id": "8a4a79fac35d414f9774ddd237a09eb155ad5203",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9955084a05d75819f74dff651c10a063ef2f5b65"
        },
        "date": 1785289529512,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 462477.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 581062.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 395046.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 348070.9,
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
          "id": "92cc54c6635968e5797ada9b806b5f74ab0a8e93",
          "message": "Merge pull request #235 from LoveDaisy/fix/install-hooks-worktree\n\nfix(install-hooks): resolve the hooks dir via git rev-parse --git-path",
          "timestamp": "2026-07-29T09:53:29+08:00",
          "tree_id": "ef5b6fe4f9f35d8c7d6032a1ad0e8c8fbc9a64df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/92cc54c6635968e5797ada9b806b5f74ab0a8e93"
        },
        "date": 1785290505950,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 483331.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583536.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400634.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 350614.9,
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
          "id": "95db17f1f62658518c88cdfd144891820be9a119",
          "message": "Merge pull request #236 from LoveDaisy/feat/pytest-invocation-gate\n\nfeat(policy): gate pytest invocations that addopts would silently empty",
          "timestamp": "2026-07-29T15:09:35+08:00",
          "tree_id": "bec8487e460ac1e3d8b09a45c4c9728f0be2162c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/95db17f1f62658518c88cdfd144891820be9a119"
        },
        "date": 1785309501631,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 444764,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 578024.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 379325,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 316421.7,
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
          "id": "86691677ff1522ce09436c46e138a863287d0e4c",
          "message": "Merge pull request #237 from LoveDaisy/feat/gui-sampling-density-stats\n\nfeat(gui): show sampling density (crystal/orientation draws) in the status bar",
          "timestamp": "2026-07-31T18:33:07+08:00",
          "tree_id": "d0ad8ee1d04f228ef63bda3518838ad181adf014",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86691677ff1522ce09436c46e138a863287d0e4c"
        },
        "date": 1785494478630,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 459500.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584499.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397180,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 317530,
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
          "id": "04a9998e115ab37676bbe9d09714b1f2240f8350",
          "message": "Merge pull request #238 from LoveDaisy/chore/regen-auto-ev-thresholds\n\nchore(gui-test): 重标定 auto_ev 组 PSNR 阈值（Phase B，N=10）",
          "timestamp": "2026-07-31T20:52:33+08:00",
          "tree_id": "0ea3eb7d2abfc5074d764d0f7d9048e62fa5462d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/04a9998e115ab37676bbe9d09714b1f2240f8350"
        },
        "date": 1785502885851,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 461611.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586383.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 379477.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 348340.6,
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
          "id": "cc8b11cdb933a3180a7d63f7dba5e9dd11a23514",
          "message": "Merge pull request #239 from LoveDaisy/fix/gui-test-realtiming-load-robustness\n\nfix(gui-test): save_open 收敛判据改为累积光线数，消除负载相关假红",
          "timestamp": "2026-08-01T07:38:52+08:00",
          "tree_id": "0daefbb9d40a4a60972bf996f79b13512022e0c5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cc8b11cdb933a3180a7d63f7dba5e9dd11a23514"
        },
        "date": 1785541709457,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 339198.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 577755.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 390843,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 344280.4,
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
          "id": "a0fd8622615b2e6f6a5ad0546e09159238614d28",
          "message": "Merge pull request #240 from LoveDaisy/fix/gui-completed-preview-quality-gate\n\nfix(gui): COMPLETED 代终帧绕过质量闸强制上屏（修有限低光线仿真预览永不出图）",
          "timestamp": "2026-08-01T12:52:25+08:00",
          "tree_id": "1c5c8c2e72f475e8aeff98efcea620a5f726d0dd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a0fd8622615b2e6f6a5ad0546e09159238614d28"
        },
        "date": 1785560477332,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 329539.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586469.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 657352,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 338869.2,
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
          "id": "f88bf70a41a987c64f3e36ce9f8ff19d102eedac",
          "message": "Merge pull request #243 from LoveDaisy/feat/gui-test-layer-cleanup\n\ntest(gui): gui_test 分层清理 —— 263 个用例迁入无窗口 gui_unit_test，首次获得 CI 覆盖",
          "timestamp": "2026-08-03T07:42:49+08:00",
          "tree_id": "efed823ff3c319c41d56da58dbf9d688687056a8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f88bf70a41a987c64f3e36ce9f8ff19d102eedac"
        },
        "date": 1785714769106,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 478467.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 578232.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 399657.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337781.9,
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
          "id": "7b8ca898ed0048b18607fdeb7c2c54890cdba249",
          "message": "Merge pull request #244 from LoveDaisy/fix/server-poller-state-reset-ownership\n\nfix(gui): ServerPoller 状态复位收敛为单一 owner + 修重启后状态栏上屏陈旧统计",
          "timestamp": "2026-08-03T11:30:30+08:00",
          "tree_id": "f9d0de83308607c4a960a1b83fe8e645030f244d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7b8ca898ed0048b18607fdeb7c2c54890cdba249"
        },
        "date": 1785728384508,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 476100.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587150.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396603.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 315523.3,
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
          "id": "c3b0c1508ca8975d18284ddfb9089722b087de4b",
          "message": "Merge pull request #245 from LoveDaisy/chore/crash-sentinel-diagnostics\n\nchore: 崩溃哨兵设施补两处诊断缺口（挂起检出 + 两臂 build 日志留痕）",
          "timestamp": "2026-08-04T07:12:15+08:00",
          "tree_id": "7f5982b56736b84800aec2024c45b53a4d40aff5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c3b0c1508ca8975d18284ddfb9089722b087de4b"
        },
        "date": 1785799323683,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 587251.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 403638.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321310.1,
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
          "id": "cf4fdfa21a63e5dc8f790812428e526294c91dcb",
          "message": "Merge pull request #246 from LoveDaisy/investigate/gui-payload-epoch-carryover\n\nfix(gui): 纹理 payload 只在内容确属当前世代时才物化",
          "timestamp": "2026-08-04T07:30:24+08:00",
          "tree_id": "309471c18a2233f37de0d05fd81efcbb05275e73",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cf4fdfa21a63e5dc8f790812428e526294c91dcb"
        },
        "date": 1785800444485,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 362741.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 577964.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397184.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 346816.7,
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
          "id": "84faab640e80563e1772399f83d2cc8c5da23afa",
          "message": "Merge pull request #247 from LoveDaisy/fix/gui-display-time-stale-payload-publish\n\nfix(gui-test): 序列化被唤醒的全局 poller，消除 display-time 编辑后的撕裂快照",
          "timestamp": "2026-08-04T10:06:23+08:00",
          "tree_id": "ba9ee6829ca13f0bb812b3833a070ae347c4fae9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/84faab640e80563e1772399f83d2cc8c5da23afa"
        },
        "date": 1785809759683,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 351885.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 580897.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 392776.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 319039.2,
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
          "id": "fb721d5b41ec6b834ab8f92634398f5a67bb73c2",
          "message": "Merge pull request #248 from LoveDaisy/explore/visual-regression-layer-value\n\ntest(gui): 退役 auto_ev 视觉回归组，把这一层接进 CI 并给「红了怎么办」写下判据",
          "timestamp": "2026-08-04T21:05:45+08:00",
          "tree_id": "07403f81e4fa76b1b60fe18689fd37d91fec211f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fb721d5b41ec6b834ab8f92634398f5a67bb73c2"
        },
        "date": 1785849353453,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 390765.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584779.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400799.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 313265.2,
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
          "id": "6e95f5c4505cac81d3efb2fbfec80b26926ff77c",
          "message": "Merge pull request #249 from LoveDaisy/explore/gui-test-suite-from-scratch\n\ndocs(testing-architecture): 新增 §4.8 —— GUI 套件形状的机制层诊断与工作规则",
          "timestamp": "2026-08-05T00:54:42+08:00",
          "tree_id": "d211f764eb7e4218ae46f8a34134e89f34aa0e31",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6e95f5c4505cac81d3efb2fbfec80b26926ff77c"
        },
        "date": 1785863083252,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 462892,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583899.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400422.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 342602,
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
          "id": "14885a4009086138c80492f57dd454c3379f93a2",
          "message": "Merge pull request #250 from LoveDaisy/feat/capi-result-lifetime-ownership\n\nrefactor(capi): 结果数据改为不可变引用计数帧 + 不透明句柄，净删六个旧 getter",
          "timestamp": "2026-08-05T10:49:05+08:00",
          "tree_id": "7d8fafa792635f949c3b78d1b503f263d2c404cc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/14885a4009086138c80492f57dd454c3379f93a2"
        },
        "date": 1785898749584,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 463159.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 579768.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 380347.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 298048.1,
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
          "id": "b21fe562ba8cdf4e0cf66f9007ee180481bfadcd",
          "message": "Merge pull request #251 from LoveDaisy/feat/preview-lifecycle-invariant-closure\n\nGUI 预览生命周期不变量族级收口（scrum-429）：I3/I4 补齐 + 新增 I7 完成蕴含排空",
          "timestamp": "2026-08-06T10:26:27+08:00",
          "tree_id": "67e60db6fdf70c87913882cb11a32d89376d886c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b21fe562ba8cdf4e0cf66f9007ee180481bfadcd"
        },
        "date": 1785983768922,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 318457.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 579518,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397350.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 318243.8,
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
          "id": "b5d4402f2d5824a55e160f6cffa7e731746c0108",
          "message": "Merge pull request #252 from LoveDaisy/chore/micro-debt-sweep\n\nchore: 微债一次结清（帧 RAII 收敛 / GUI 日志装配 / 可移植测试路径 / C API 边界证据）",
          "timestamp": "2026-08-06T12:31:23+08:00",
          "tree_id": "03aad82a20199d682144c4ce75301214e26541c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b5d4402f2d5824a55e160f6cffa7e731746c0108"
        },
        "date": 1785991302463,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 372612.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585308.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 395811.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 357173.3,
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
          "id": "5a8f8982aa3d6bf634f3f13b6cf53ccc29bce9a4",
          "message": "Merge pull request #253 from LoveDaisy/refactor/user-defaults-write-surface-closure\n\nrefactor(gui): close the parallel user-defaults write surface (434)",
          "timestamp": "2026-08-06T16:41:53+08:00",
          "tree_id": "e39098a9bc1cf37fc4ce2c511b3cc4c9e248608f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5a8f8982aa3d6bf634f3f13b6cf53ccc29bce9a4"
        },
        "date": 1786006346411,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 452681.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583957,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 609164.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 353361.3,
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
          "id": "4e3f2bf4a1010ee5fe92f5a0cb24a863a3d96272",
          "message": "Merge pull request #254 from LoveDaisy/fix/revert-field-scope-alignment\n\nfix(gui): align Revert's field scope with the predicate that decides what counts as a change",
          "timestamp": "2026-08-06T17:07:01+08:00",
          "tree_id": "39a256ded94aa63c7c11aaeb8f2c923af34401c7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4e3f2bf4a1010ee5fe92f5a0cb24a863a3d96272"
        },
        "date": 1786007854448,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 337434.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592328.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398755.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 304469,
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
          "id": "559b976cfc69c740bfbb7bcf1a4c9f0bac592e3c",
          "message": "Merge pull request #255 from LoveDaisy/feat/gui-rules-as-data\n\nMake three GUI rules queryable data, and replace the grid tests they forced",
          "timestamp": "2026-08-07T07:52:53+08:00",
          "tree_id": "e263eeb0da176a674de554034f5744e15cc45bf6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/559b976cfc69c740bfbb7bcf1a4c9f0bac592e3c"
        },
        "date": 1786060883050,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 329767.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582938.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396476.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 415972.9,
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
          "id": "985357aace8cb86e666f1c4e317f08c847f2af51",
          "message": "Merge pull request #256 from LoveDaisy/fix/pyramid-closed-form-geometry-defects\n\nfix(core): close the closed-form pyramid's structural geometry defects",
          "timestamp": "2026-08-07T14:54:11+08:00",
          "tree_id": "08e78fc2e34f33e8a8b828d75ac37a0294dd0cad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/985357aace8cb86e666f1c4e317f08c847f2af51"
        },
        "date": 1786086317801,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 317555.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 577357.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 395410,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 344849.6,
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
          "id": "6b3c6b4ca3743e19c327b64957e7c30aea188970",
          "message": "Merge pull request #257 from LoveDaisy/feat/config-default-semantics\n\nMake core's implicit config defaults into written contracts (prob / axis type / absent axis)",
          "timestamp": "2026-08-07T15:41:10+08:00",
          "tree_id": "8426a314fa4cb0aff7bfe853eb95de84ef4b10f0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6b3c6b4ca3743e19c327b64957e7c30aea188970"
        },
        "date": 1786089125484,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 377758.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 585954.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389864.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345726.6,
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
          "id": "35c7cd3f1734523f8812d509152a6b3ff8ab2ea7",
          "message": "Merge pull request #258 from LoveDaisy/fix/preview-drag-gain-fov\n\nfix(gui): scale preview drag by the lens's angular resolution",
          "timestamp": "2026-08-07T16:10:31+08:00",
          "tree_id": "91628e3757321fdf8723aac188f3c9c5be6aa3e5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35c7cd3f1734523f8812d509152a6b3ff8ab2ea7"
        },
        "date": 1786090773647,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 292755.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 590715.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396578.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 395634.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
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
          "id": "f1fab1851731a56919ca3ade95ffca37f3ea8d01",
          "message": "build(deps): bump Jimver/cuda-toolkit from 0.2.35 to 0.2.36\n\nBumps [Jimver/cuda-toolkit](https://github.com/jimver/cuda-toolkit) from 0.2.35 to 0.2.36.\n- [Release notes](https://github.com/jimver/cuda-toolkit/releases)\n- [Commits](https://github.com/jimver/cuda-toolkit/compare/v0.2.35...v0.2.36)\n\n---\nupdated-dependencies:\n- dependency-name: Jimver/cuda-toolkit\n  dependency-version: 0.2.36\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-08-08T08:35:42+08:00",
          "tree_id": "e00d185b3a3bebca6b8e9c131180275191f5fc28",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1fab1851731a56919ca3ade95ffca37f3ea8d01"
        },
        "date": 1786149837342,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 360973,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583122.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400707.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 549995,
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
          "id": "e9c3c21619ce17f2552afeae2398ef33fbce4ee4",
          "message": "Merge pull request #260 from LoveDaisy/fix/closedform-tolerance-residuals\n\nfix(core): remove the closed-form pyramid's unit-of-measure assumption from its tolerances",
          "timestamp": "2026-08-08T12:02:24+08:00",
          "tree_id": "9f892f4c5b6e1fef411a3819fa50d9ab8f4ccfe7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e9c3c21619ce17f2552afeae2398ef33fbce4ee4"
        },
        "date": 1786162362102,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 449355.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 583483,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396663.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 392736.2,
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
          "id": "e9f15c07a86a7eb9878e5a43e149b5137e846df3",
          "message": "Merge pull request #261 from LoveDaisy/feat/gui-test-suite-rebuild\n\ntest(gui): rebuild the GUI test suite into three layers, and gate the cascade defect family",
          "timestamp": "2026-08-10T22:24:25+08:00",
          "tree_id": "e9887d79b9e35fe3454a3514c13308ca2f68fc03",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e9f15c07a86a7eb9878e5a43e149b5137e846df3"
        },
        "date": 1786372554954,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 349235.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586281.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 379331.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321851.5,
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
          "id": "5c2b1a9cd1daae6d11f8b670faca311a48723059",
          "message": "Merge pull request #262 from LoveDaisy/fix/gui-blocked-production-defects\n\nfix(gui): four defects around blank filter rows, slider drags and failed loads",
          "timestamp": "2026-08-11T11:22:23+08:00",
          "tree_id": "86c4753b809868150c88ccfedc5bd8d90ea97394",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c2b1a9cd1daae6d11f8b670faca311a48723059"
        },
        "date": 1786419206772,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 438791.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 578699.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 674302.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 303719,
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
          "id": "1a021d0025f312751971b24b6417dc1662c14555",
          "message": "Merge pull request #263 from LoveDaisy/chore/doc-stale-state-claims\n\nMake the docs and comments say what the code actually does now",
          "timestamp": "2026-08-11T12:09:28+08:00",
          "tree_id": "1cb6721b617dd3dce5b8306763e1828b8317f689",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1a021d0025f312751971b24b6417dc1662c14555"
        },
        "date": 1786422061116,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 336450.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586399.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 391076.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 350553.6,
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
          "id": "6ee81825abca90286b1eaef923f6aeaaaabae056",
          "message": "Merge pull request #202 from LoveDaisy/feat/filter-form-big-or\n\nfeat: 放开 filter OR-clause 上限 8/16→4096(纯过滤,染色 mask 不动)",
          "timestamp": "2026-07-16T07:43:52+08:00",
          "tree_id": "c02e2eebc9625ec984d3d85fbb779cf8f25ea060",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6ee81825abca90286b1eaef923f6aeaaaabae056"
        },
        "date": 1784159461763,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1057334.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1167219.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 741004.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 692011.2,
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
          "id": "ec1879077f613c9bbfaabb3079281ef5164bb2de",
          "message": "Merge pull request #203 from LoveDaisy/chore/reconciler-gate-wake-helper\n\nchore: harden reconciler include boundary + dedup wake path (scrum-353 T2 follow-up)",
          "timestamp": "2026-07-16T08:44:05+08:00",
          "tree_id": "b12a3a7a06d99002a3d740ef5c655ebef8581689",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ec1879077f613c9bbfaabb3079281ef5164bb2de"
        },
        "date": 1784163006052,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 915733.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1181252.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 735577.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 900844.7,
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
          "id": "c89c178f203a9956ee50c8af21c7f0300615d053",
          "message": "Merge pull request #204 from LoveDaisy/feat/color-degrade-gui-surfacing\n\nfeat(color-degrade-gui-surfacing): surface all 3 GPU color-degrade caps to GUI modal",
          "timestamp": "2026-07-16T11:53:31+08:00",
          "tree_id": "ac0cf27c253b945d4191e4e914f50f1ad145a0fe",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c89c178f203a9956ee50c8af21c7f0300615d053"
        },
        "date": 1784174450736,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1171739.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1179531.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1181815.6,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 663815.4,
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
          "id": "ab7aa0b509b61ae45afbf6b3f6fb7846507c53b1",
          "message": "Merge pull request #205 from LoveDaisy/chore/policy-gates\n\nchore(policy-gates): fix the bench compile rot and gate working-note references",
          "timestamp": "2026-07-17T07:52:34+08:00",
          "tree_id": "c40692428310e7ecfc1ae1acafbfeb140902be36",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ab7aa0b509b61ae45afbf6b3f6fb7846507c53b1"
        },
        "date": 1784246399106,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1172231.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1183994.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 779761.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 655334.4,
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
          "id": "a610dbde93b0957805c532933a5bde34fabc21a2",
          "message": "Merge pull request #206 from LoveDaisy/fix/degenerate-geometry\n\nfix(core): random face_distance SIGSEGV — scale-relative vertex dedup + non-manifold rejection",
          "timestamp": "2026-07-17T15:08:47+08:00",
          "tree_id": "771c0d3b33012a43a90f8696bf92c5510957945e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a610dbde93b0957805c532933a5bde34fabc21a2"
        },
        "date": 1784272568737,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1214820.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1171462,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 741242,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 662541.8,
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
          "id": "489bfb6288808f55579d34cc8361b00fd84d8fb0",
          "message": "Merge pull request #207 from LoveDaisy/docs/geom-clock-and-benchmark-caveats\n\ndocs: correct two measurement caveats found while calibrating the geometry clock",
          "timestamp": "2026-07-17T18:00:16+08:00",
          "tree_id": "56bd0351ee4a8d7916c831f46b0b8cc6b5ce0f59",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/489bfb6288808f55579d34cc8361b00fd84d8fb0"
        },
        "date": 1784282850732,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1048620.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1185610.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 740471.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 591780.5,
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
          "id": "e22f9eea8e5d88a52b44452e88488b73c5bc9ec0",
          "message": "Merge pull request #208 from LoveDaisy/fix/pyramid-geometry-crash-metal\n\nfix(core): pyramid + random face_distance Metal SIGSEGV (count/stride decouple)",
          "timestamp": "2026-07-18T12:55:41+08:00",
          "tree_id": "4664a91a1f187e7cb56fae546dfaa87d149d926c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e22f9eea8e5d88a52b44452e88488b73c5bc9ec0"
        },
        "date": 1784350990787,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 1183958.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 951964.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 663849.1,
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
          "id": "f29881b3f101b6c06279e10954d789f5984d27bb",
          "message": "Merge pull request #210 from LoveDaisy/chore/gbk-locale-parity-test-unicode\n\nfix(test): ASCII-ize parity test messages for GBK-locale Windows",
          "timestamp": "2026-07-19T09:17:00+08:00",
          "tree_id": "ec1b7089cc6b1d84ac772796a218885ef9dfbe58",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f29881b3f101b6c06279e10954d789f5984d27bb"
        },
        "date": 1784424298837,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1094052,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1188377.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 775101.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 651496.4,
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
          "id": "abbef1227936e40b1753817142344886fae0de78",
          "message": "Merge pull request #209 from LoveDaisy/fix/cuda-unfreeze-geometry-randomization\n\nfix(cuda): unfreeze crystal-shape geometry randomization end-to-end",
          "timestamp": "2026-07-19T09:16:57+08:00",
          "tree_id": "836469c0bb3baf78eec16a14d2eb5e42760849c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/abbef1227936e40b1753817142344886fae0de78"
        },
        "date": 1784425810032,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1137269.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1179253.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 772951.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 655806.4,
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
          "id": "d9d7ee2ff6accd887235198435abfedf87e4f1b2",
          "message": "Merge pull request #212 from LoveDaisy/chore/fix-base\n\nChore/fix base",
          "timestamp": "2026-07-19T10:15:27+08:00",
          "tree_id": "63532f0f8cbe507abecaa438c81d729aee904549",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d9d7ee2ff6accd887235198435abfedf87e4f1b2"
        },
        "date": 1784427801367,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1183204.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1193312.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 772982.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 659250.5,
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
          "id": "163ff642d86a80c43b8197a2ba57a5016cd6d6a7",
          "message": "Merge pull request #213 from LoveDaisy/feat/geometry-pool-and-topology-reuse\n\nfeat(geometry-perf): per-ray K-shape pool on both GPU backends + geometry representation diagnosis",
          "timestamp": "2026-07-20T11:37:25+08:00",
          "tree_id": "099175e5599952910c54ce6d8ff31717f756c00f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/163ff642d86a80c43b8197a2ba57a5016cd6d6a7"
        },
        "date": 1784519097844,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 940562.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1171294,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 781183.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 570836.2,
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
          "id": "4057ce13d077863ae1027d799f1fe8d4c8fe1cb0",
          "message": "Merge pull request #214 from LoveDaisy/feat/geometry-closed-form-representation\n\nfeat(geometry): closed-form hex crystal representation (scrum-386)",
          "timestamp": "2026-07-21T13:25:10+08:00",
          "tree_id": "03e5310167743bc37a0d3c76b1b4600e8b4983c0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4057ce13d077863ae1027d799f1fe8d4c8fe1cb0"
        },
        "date": 1784611973872,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1097407.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175279.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 770432.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 653822.4,
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
          "id": "0244b5f169af36f52d1fbfcf83ea3ca4df809f12",
          "message": "Merge pull request #215 from LoveDaisy/feat/geometry-exact-domain-audit\n\ngeometry exactness: symbolic-a1 exact oracle (drop __int128) + pyramid apex bug fix + 3-platform verify",
          "timestamp": "2026-07-22T15:28:14+08:00",
          "tree_id": "455c7b6b9c07b5fffdb7b5cc4cd6c4c0844c0eb7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/0244b5f169af36f52d1fbfcf83ea3ca4df809f12"
        },
        "date": 1784705727772,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 881831.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1192044.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 780663.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 606069,
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
          "id": "7bd2f401246d724694c7c02bbce7a37b093f43d4",
          "message": "Merge pull request #216 from LoveDaisy/feat/pyramid-oracle-contract-tests\n\ngeometry test: retire symbolic-α pyramid oracle for three contract-aligned tests",
          "timestamp": "2026-07-22T17:35:30+08:00",
          "tree_id": "316a03a93602e9c23a00832d413abf5c51648733",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7bd2f401246d724694c7c02bbce7a37b093f43d4"
        },
        "date": 1784713397058,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1060155.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1182538.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 743334.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 653193.6,
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
          "id": "de7b4571c336a6d6f4814657c8822c6f149fb091",
          "message": "Merge pull request #217 from LoveDaisy/feat/geom-pool-metal-landing\n\nfeat(geom-pool): wire K-shape pool geom_clock into config + Metal/CUDA backends",
          "timestamp": "2026-07-22T23:04:47+08:00",
          "tree_id": "69bd58055ec2bb19e867aaf6a9ac66a07a16cb55",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de7b4571c336a6d6f4814657c8822c6f149fb091"
        },
        "date": 1784733124457,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 866540.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1194165.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 958662.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 604182.1,
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
          "id": "c527f60f46b67a4c4bb8bab34f73281efb072b65",
          "message": "Merge pull request #218 from LoveDaisy/feat/cuda-degenerate-geometry-parity\n\nfeat(cuda): degenerate K-shape pool parity + crystal-count assertion (scrum-392)",
          "timestamp": "2026-07-23T12:17:55+08:00",
          "tree_id": "0127101471b6d7dd59e5faea4bc8329d73e60df3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c527f60f46b67a4c4bb8bab34f73281efb072b65"
        },
        "date": 1784780744055,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 885629.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1195890.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 779638.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 654981.5,
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
          "id": "86b954890e231d91593eefe4a83ec335fe440227",
          "message": "Merge pull request #219 from LoveDaisy/feat/crystal-consumption-detriangulation\n\nfeat(core): detriangulate crystal consumption — polygon-granularity incidence sampling, remove triangle mesh from hot path",
          "timestamp": "2026-07-24T08:25:14+08:00",
          "tree_id": "7b83d1927fad720bef607c05a2de71ac1331ef3a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86b954890e231d91593eefe4a83ec335fe440227"
        },
        "date": 1784853153416,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1026605.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1166708.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 730354.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 552107.6,
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
          "id": "1459b4b094c37b2dd13657b05447d4a342cb6d1b",
          "message": "Merge pull request #220 from LoveDaisy/feat/strong-randomization-downstream-contracts\n\nfix: strong-randomization downstream contracts (filter/render/consumer)",
          "timestamp": "2026-07-24T18:21:13+08:00",
          "tree_id": "418f71e6a27d0ed077d6af076ab0f1ef0f95ab01",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1459b4b094c37b2dd13657b05447d4a342cb6d1b"
        },
        "date": 1784888925834,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 853996.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175138.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 768043,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 536360.6,
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
          "id": "85e12c4e4ae37fe7ffb660b9e22cce0eb96f71c3",
          "message": "Merge pull request #221 from LoveDaisy/feat/core-distribution-cleanup\n\nfeat: crystal shape randomization in GUI + first-class LUMICE_Distribution (BREAKING v4.10)",
          "timestamp": "2026-07-24T20:13:16+08:00",
          "tree_id": "32f4fc909d8d071013c9f3cfd519a135f6cfca2d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85e12c4e4ae37fe7ffb660b9e22cce0eb96f71c3"
        },
        "date": 1784895678411,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 728626.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1173972.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 767454.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 667063.6,
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
          "id": "fa613fb092061df691f456e4ee68497590efe136",
          "message": "Merge pull request #223 from LoveDaisy/feat/gui-shape-randomization-property-table\n\nfeat(gui): crystal shape randomization as a single uniform property table",
          "timestamp": "2026-07-25T09:15:06+08:00",
          "tree_id": "e97db9aed21e5466cd569d61f8ad02416099e8ee",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa613fb092061df691f456e4ee68497590efe136"
        },
        "date": 1784942602768,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 820446.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1176449.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 766127.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 615838,
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
          "id": "86adcd8f5fc57d211e201db64fb8d77f4bf37f88",
          "message": "Merge pull request #224 from LoveDaisy/feat/capi-scene-opaque-handle\n\nrefactor(c_api): LUMICE_Config value struct → LUMICE_Scene opaque handle (BREAKING v4.12)",
          "timestamp": "2026-07-25T21:38:42+08:00",
          "tree_id": "c1c944a06dc5cd04ab420b22aae79f66f9a92ced",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86adcd8f5fc57d211e201db64fb8d77f4bf37f88"
        },
        "date": 1784987163738,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1127932.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1178595.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 713553.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 708658,
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
          "id": "62ba3d6ba31990e43588d1b8c222716cc1a70d5d",
          "message": "Merge pull request #225 from LoveDaisy/feat/gui-visual-regression-coverage\n\ntest(gui): reference-image pixel regression for lens projections + modal layouts",
          "timestamp": "2026-07-25T22:04:33+08:00",
          "tree_id": "750dc3e4cce3891a10ee627c0a5fde4efbbc3a21",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/62ba3d6ba31990e43588d1b8c222716cc1a70d5d"
        },
        "date": 1784988680199,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1112559,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175136.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 723039.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 799730.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
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
          "id": "55bfc00af291844e9883a3a68e4c7ac4bc9a8bb3",
          "message": "chore(deps): bump actions/setup-python from 6 to 7\n\nBumps [actions/setup-python](https://github.com/actions/setup-python) from 6 to 7.\n- [Release notes](https://github.com/actions/setup-python/releases)\n- [Commits](https://github.com/actions/setup-python/compare/v6...v7)\n\n---\nupdated-dependencies:\n- dependency-name: actions/setup-python\n  dependency-version: '7'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-07-25T22:18:22+08:00",
          "tree_id": "f42b07d44e062a9971d1edbbc7b7be647d058b05",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/55bfc00af291844e9883a3a68e4c7ac4bc9a8bb3"
        },
        "date": 1784989587839,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1109289,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1174239.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 762079.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 651794,
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
          "id": "28e5a9de538df35ed7b5b980592cd051c1109e71",
          "message": "Merge pull request #226 from LoveDaisy/chore/dead-weight-closeout\n\nchore(dead-weight-closeout): delete orphaned Server::CommitConfigFromFile, fix stale ExitRayRecord size comments",
          "timestamp": "2026-07-25T23:16:52+08:00",
          "tree_id": "6622931ec23b0c2436e0728a5bdcc31f2e48b31d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/28e5a9de538df35ed7b5b980592cd051c1109e71"
        },
        "date": 1784993015626,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1184715.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1179661.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 768817.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 676207.5,
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
          "id": "b8248749346e73a9c28c6d9b8c8e68d688f477bc",
          "message": "Merge pull request #227 from LoveDaisy/chore/unified-logging-gate\n\nchore: route src/ diagnostics through the logger, gate bare prints",
          "timestamp": "2026-07-26T00:33:32+08:00",
          "tree_id": "778d66a5d877fafaa592ec918b1102342dcec321",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b8248749346e73a9c28c6d9b8c8e68d688f477bc"
        },
        "date": 1784997689320,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1070531,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172839.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 858534.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 611367.8,
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
          "id": "9b530515528d848ef297c39f68be7d2b0db36841",
          "message": "Merge pull request #228 from LoveDaisy/feat/face-distance-sync-groups\n\nfeat: shape-scalar sync groups (symmetry-preserving shape randomization)",
          "timestamp": "2026-07-27T14:08:08+08:00",
          "tree_id": "e8c7e3774d16d984526ed00e175883c3166fbb79",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9b530515528d848ef297c39f68be7d2b0db36841"
        },
        "date": 1785133051427,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 759609.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1182283.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 759437.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 638750,
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
          "id": "1104a952b08241bd61e45bcf7380439a761cbbf7",
          "message": "Merge pull request #229 from LoveDaisy/feat/crystal-sample-count-semantics\n\nfix(stats): make crystals=N a scene property instead of a schedule artifact",
          "timestamp": "2026-07-27T15:17:09+08:00",
          "tree_id": "330fc333a3c88c2f6fcf80ba8e69d53bbf64b031",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1104a952b08241bd61e45bcf7380439a761cbbf7"
        },
        "date": 1785137105126,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 927242.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1171786.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 772498.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 605584.7,
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
          "id": "ce07b6aea91996dafbf320883cdcd408d861b672",
          "message": "Merge pull request #230 from LoveDaisy/feat/shape-schema-key-single-source\n\nrefactor: give the crystal shape/axis JSON keys one owner in core",
          "timestamp": "2026-07-27T17:39:30+08:00",
          "tree_id": "e1b5c4d911db4ddbe74f30e0e534b9eed7be52b1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ce07b6aea91996dafbf320883cdcd408d861b672"
        },
        "date": 1785145671467,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1121510.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175935.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 931492.9,
            "unit": "rays/sec",
            "extra": "CPU: INTEL(R) XEON(R) PLATINUM 8573C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 647778.9,
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
          "id": "19ff683644d3f8e69afc7f9c3226be0b5971d9b1",
          "message": "Merge pull request #231 from LoveDaisy/feat/user-defaults\n\nfeat(gui): user-level defaults layer — generated diff panel + editable axis preset library",
          "timestamp": "2026-07-28T09:03:52+08:00",
          "tree_id": "705f62fb463dfc4fa6d35c7c57aa2735e68ff882",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/19ff683644d3f8e69afc7f9c3226be0b5971d9b1"
        },
        "date": 1785201083772,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 822601.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1180891.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 727839.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1118176.2,
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
          "id": "a9d95c29cf1c41e15ecd018702019bef1d7f8f5d",
          "message": "Merge pull request #232 from LoveDaisy/feat/orientation-sample-count-stat\n\nfeat(stats): report orientation sample count as an independent statistic",
          "timestamp": "2026-07-28T13:40:08+08:00",
          "tree_id": "eac8e9336b7adffc48a245cc5cf3dc47ddc6764c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a9d95c29cf1c41e15ecd018702019bef1d7f8f5d"
        },
        "date": 1785217725047,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 819431.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1165657.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 767479.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 643171.9,
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
          "id": "c8492857cbb930e8415baf8267aaadf2082f7b6e",
          "message": "Merge pull request #233 from LoveDaisy/feat/local-test-scope-and-docs\n\nbuild: per-flavor build/install trees + gate bare pytest to the fast subset",
          "timestamp": "2026-07-29T09:07:07+08:00",
          "tree_id": "7796a30d82f61ff9812359a3848ac7bbeb8ccc0a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c8492857cbb930e8415baf8267aaadf2082f7b6e"
        },
        "date": 1785287716991,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 978761.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1160848.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 758876.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 601717.6,
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
          "id": "9955084a05d75819f74dff651c10a063ef2f5b65",
          "message": "Merge pull request #234 from LoveDaisy/feat/panel-settings-editor\n\nfeat(gui): make the defaults panel a pure editor with one source of constraint truth",
          "timestamp": "2026-07-29T09:36:30+08:00",
          "tree_id": "8a4a79fac35d414f9774ddd237a09eb155ad5203",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9955084a05d75819f74dff651c10a063ef2f5b65"
        },
        "date": 1785289532593,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1104545.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1155789.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 769497.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 639890.8,
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
          "id": "92cc54c6635968e5797ada9b806b5f74ab0a8e93",
          "message": "Merge pull request #235 from LoveDaisy/fix/install-hooks-worktree\n\nfix(install-hooks): resolve the hooks dir via git rev-parse --git-path",
          "timestamp": "2026-07-29T09:53:29+08:00",
          "tree_id": "ef5b6fe4f9f35d8c7d6032a1ad0e8c8fbc9a64df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/92cc54c6635968e5797ada9b806b5f74ab0a8e93"
        },
        "date": 1785290510034,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1178184,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1177514.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 771824.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 654789.7,
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
          "id": "95db17f1f62658518c88cdfd144891820be9a119",
          "message": "Merge pull request #236 from LoveDaisy/feat/pytest-invocation-gate\n\nfeat(policy): gate pytest invocations that addopts would silently empty",
          "timestamp": "2026-07-29T15:09:35+08:00",
          "tree_id": "bec8487e460ac1e3d8b09a45c4c9728f0be2162c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/95db17f1f62658518c88cdfd144891820be9a119"
        },
        "date": 1785309505344,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1162009.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1168994.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 727332.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 584233.7,
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
          "id": "86691677ff1522ce09436c46e138a863287d0e4c",
          "message": "Merge pull request #237 from LoveDaisy/feat/gui-sampling-density-stats\n\nfeat(gui): show sampling density (crystal/orientation draws) in the status bar",
          "timestamp": "2026-07-31T18:33:07+08:00",
          "tree_id": "d0ad8ee1d04f228ef63bda3518838ad181adf014",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86691677ff1522ce09436c46e138a863287d0e4c"
        },
        "date": 1785494482713,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1029246.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1159821.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 768526.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 610250.7,
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
          "id": "04a9998e115ab37676bbe9d09714b1f2240f8350",
          "message": "Merge pull request #238 from LoveDaisy/chore/regen-auto-ev-thresholds\n\nchore(gui-test): 重标定 auto_ev 组 PSNR 阈值（Phase B，N=10）",
          "timestamp": "2026-07-31T20:52:33+08:00",
          "tree_id": "0ea3eb7d2abfc5074d764d0f7d9048e62fa5462d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/04a9998e115ab37676bbe9d09714b1f2240f8350"
        },
        "date": 1785502890821,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1145261.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1167690.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 727616,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 619605,
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
          "id": "cc8b11cdb933a3180a7d63f7dba5e9dd11a23514",
          "message": "Merge pull request #239 from LoveDaisy/fix/gui-test-realtiming-load-robustness\n\nfix(gui-test): save_open 收敛判据改为累积光线数，消除负载相关假红",
          "timestamp": "2026-08-01T07:38:52+08:00",
          "tree_id": "0daefbb9d40a4a60972bf996f79b13512022e0c5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cc8b11cdb933a3180a7d63f7dba5e9dd11a23514"
        },
        "date": 1785541713781,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 780120.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175255,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 770156.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 615554.1,
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
          "id": "a0fd8622615b2e6f6a5ad0546e09159238614d28",
          "message": "Merge pull request #240 from LoveDaisy/fix/gui-completed-preview-quality-gate\n\nfix(gui): COMPLETED 代终帧绕过质量闸强制上屏（修有限低光线仿真预览永不出图）",
          "timestamp": "2026-08-01T12:52:25+08:00",
          "tree_id": "1c5c8c2e72f475e8aeff98efcea620a5f726d0dd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a0fd8622615b2e6f6a5ad0546e09159238614d28"
        },
        "date": 1785560480957,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 821030.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1182069.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1261341.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 633901.6,
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
          "id": "f88bf70a41a987c64f3e36ce9f8ff19d102eedac",
          "message": "Merge pull request #243 from LoveDaisy/feat/gui-test-layer-cleanup\n\ntest(gui): gui_test 分层清理 —— 263 个用例迁入无窗口 gui_unit_test，首次获得 CI 覆盖",
          "timestamp": "2026-08-03T07:42:49+08:00",
          "tree_id": "efed823ff3c319c41d56da58dbf9d688687056a8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f88bf70a41a987c64f3e36ce9f8ff19d102eedac"
        },
        "date": 1785714773103,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1182603.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1170011.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 772905.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 639187.7,
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
          "id": "7b8ca898ed0048b18607fdeb7c2c54890cdba249",
          "message": "Merge pull request #244 from LoveDaisy/fix/server-poller-state-reset-ownership\n\nfix(gui): ServerPoller 状态复位收敛为单一 owner + 修重启后状态栏上屏陈旧统计",
          "timestamp": "2026-08-03T11:30:30+08:00",
          "tree_id": "f9d0de83308607c4a960a1b83fe8e645030f244d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7b8ca898ed0048b18607fdeb7c2c54890cdba249"
        },
        "date": 1785728388563,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1234816.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1180801.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 773623.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 603774.1,
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
          "id": "c3b0c1508ca8975d18284ddfb9089722b087de4b",
          "message": "Merge pull request #245 from LoveDaisy/chore/crash-sentinel-diagnostics\n\nchore: 崩溃哨兵设施补两处诊断缺口（挂起检出 + 两臂 build 日志留痕）",
          "timestamp": "2026-08-04T07:12:15+08:00",
          "tree_id": "7f5982b56736b84800aec2024c45b53a4d40aff5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c3b0c1508ca8975d18284ddfb9089722b087de4b"
        },
        "date": 1785799326798,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 1166309.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 769799.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 607642.3,
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
          "id": "cf4fdfa21a63e5dc8f790812428e526294c91dcb",
          "message": "Merge pull request #246 from LoveDaisy/investigate/gui-payload-epoch-carryover\n\nfix(gui): 纹理 payload 只在内容确属当前世代时才物化",
          "timestamp": "2026-08-04T07:30:24+08:00",
          "tree_id": "309471c18a2233f37de0d05fd81efcbb05275e73",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cf4fdfa21a63e5dc8f790812428e526294c91dcb"
        },
        "date": 1785800449306,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 896330.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1167613,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 771230.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 649976.7,
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
          "id": "84faab640e80563e1772399f83d2cc8c5da23afa",
          "message": "Merge pull request #247 from LoveDaisy/fix/gui-display-time-stale-payload-publish\n\nfix(gui-test): 序列化被唤醒的全局 poller，消除 display-time 编辑后的撕裂快照",
          "timestamp": "2026-08-04T10:06:23+08:00",
          "tree_id": "ba9ee6829ca13f0bb812b3833a070ae347c4fae9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/84faab640e80563e1772399f83d2cc8c5da23afa"
        },
        "date": 1785809763582,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 780835.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1168949.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 767241,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 602602.5,
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
          "id": "fb721d5b41ec6b834ab8f92634398f5a67bb73c2",
          "message": "Merge pull request #248 from LoveDaisy/explore/visual-regression-layer-value\n\ntest(gui): 退役 auto_ev 视觉回归组，把这一层接进 CI 并给「红了怎么办」写下判据",
          "timestamp": "2026-08-04T21:05:45+08:00",
          "tree_id": "07403f81e4fa76b1b60fe18689fd37d91fec211f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fb721d5b41ec6b834ab8f92634398f5a67bb73c2"
        },
        "date": 1785849358096,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 951734,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1174180.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 769366.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 594693.5,
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
          "id": "6e95f5c4505cac81d3efb2fbfec80b26926ff77c",
          "message": "Merge pull request #249 from LoveDaisy/explore/gui-test-suite-from-scratch\n\ndocs(testing-architecture): 新增 §4.8 —— GUI 套件形状的机制层诊断与工作规则",
          "timestamp": "2026-08-05T00:54:42+08:00",
          "tree_id": "d211f764eb7e4218ae46f8a34134e89f34aa0e31",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6e95f5c4505cac81d3efb2fbfec80b26926ff77c"
        },
        "date": 1785863087252,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1079503.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1181499.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 770857.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 654824.2,
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
          "id": "14885a4009086138c80492f57dd454c3379f93a2",
          "message": "Merge pull request #250 from LoveDaisy/feat/capi-result-lifetime-ownership\n\nrefactor(capi): 结果数据改为不可变引用计数帧 + 不透明句柄，净删六个旧 getter",
          "timestamp": "2026-08-05T10:49:05+08:00",
          "tree_id": "7d8fafa792635f949c3b78d1b503f263d2c404cc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/14885a4009086138c80492f57dd454c3379f93a2"
        },
        "date": 1785898753086,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1132290.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1170735,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 727030.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 563396.8,
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
          "id": "b21fe562ba8cdf4e0cf66f9007ee180481bfadcd",
          "message": "Merge pull request #251 from LoveDaisy/feat/preview-lifecycle-invariant-closure\n\nGUI 预览生命周期不变量族级收口（scrum-429）：I3/I4 补齐 + 新增 I7 完成蕴含排空",
          "timestamp": "2026-08-06T10:26:27+08:00",
          "tree_id": "67e60db6fdf70c87913882cb11a32d89376d886c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b21fe562ba8cdf4e0cf66f9007ee180481bfadcd"
        },
        "date": 1785983773252,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 845300.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1149011.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 767615.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 612302.2,
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
          "id": "b5d4402f2d5824a55e160f6cffa7e731746c0108",
          "message": "Merge pull request #252 from LoveDaisy/chore/micro-debt-sweep\n\nchore: 微债一次结清（帧 RAII 收敛 / GUI 日志装配 / 可移植测试路径 / C API 边界证据）",
          "timestamp": "2026-08-06T12:31:23+08:00",
          "tree_id": "03aad82a20199d682144c4ce75301214e26541c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b5d4402f2d5824a55e160f6cffa7e731746c0108"
        },
        "date": 1785991305558,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 823026.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1162802.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 770496.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 664106,
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
          "id": "5a8f8982aa3d6bf634f3f13b6cf53ccc29bce9a4",
          "message": "Merge pull request #253 from LoveDaisy/refactor/user-defaults-write-surface-closure\n\nrefactor(gui): close the parallel user-defaults write surface (434)",
          "timestamp": "2026-08-06T16:41:53+08:00",
          "tree_id": "e39098a9bc1cf37fc4ce2c511b3cc4c9e248608f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5a8f8982aa3d6bf634f3f13b6cf53ccc29bce9a4"
        },
        "date": 1786006349874,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1111154.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172075,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1147018.7,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 661961,
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
          "id": "4e3f2bf4a1010ee5fe92f5a0cb24a863a3d96272",
          "message": "Merge pull request #254 from LoveDaisy/fix/revert-field-scope-alignment\n\nfix(gui): align Revert's field scope with the predicate that decides what counts as a change",
          "timestamp": "2026-08-06T17:07:01+08:00",
          "tree_id": "39a256ded94aa63c7c11aaeb8f2c923af34401c7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4e3f2bf4a1010ee5fe92f5a0cb24a863a3d96272"
        },
        "date": 1786007857934,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 828600.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1169435.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 774296.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 555233.7,
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
          "id": "559b976cfc69c740bfbb7bcf1a4c9f0bac592e3c",
          "message": "Merge pull request #255 from LoveDaisy/feat/gui-rules-as-data\n\nMake three GUI rules queryable data, and replace the grid tests they forced",
          "timestamp": "2026-08-07T07:52:53+08:00",
          "tree_id": "e263eeb0da176a674de554034f5744e15cc45bf6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/559b976cfc69c740bfbb7bcf1a4c9f0bac592e3c"
        },
        "date": 1786060887343,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 786890.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1167970.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 773586.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 783611.9,
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
          "id": "985357aace8cb86e666f1c4e317f08c847f2af51",
          "message": "Merge pull request #256 from LoveDaisy/fix/pyramid-closed-form-geometry-defects\n\nfix(core): close the closed-form pyramid's structural geometry defects",
          "timestamp": "2026-08-07T14:54:11+08:00",
          "tree_id": "08e78fc2e34f33e8a8b828d75ac37a0294dd0cad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/985357aace8cb86e666f1c4e317f08c847f2af51"
        },
        "date": 1786086322222,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 756366.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1160904,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 768701.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 635614.3,
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
          "id": "6b3c6b4ca3743e19c327b64957e7c30aea188970",
          "message": "Merge pull request #257 from LoveDaisy/feat/config-default-semantics\n\nMake core's implicit config defaults into written contracts (prob / axis type / absent axis)",
          "timestamp": "2026-08-07T15:41:10+08:00",
          "tree_id": "8426a314fa4cb0aff7bfe853eb95de84ef4b10f0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6b3c6b4ca3743e19c327b64957e7c30aea188970"
        },
        "date": 1786089129013,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 897892.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172841.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 763477.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 655729.3,
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
          "id": "35c7cd3f1734523f8812d509152a6b3ff8ab2ea7",
          "message": "Merge pull request #258 from LoveDaisy/fix/preview-drag-gain-fov\n\nfix(gui): scale preview drag by the lens's angular resolution",
          "timestamp": "2026-08-07T16:10:31+08:00",
          "tree_id": "91628e3757321fdf8723aac188f3c9c5be6aa3e5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35c7cd3f1734523f8812d509152a6b3ff8ab2ea7"
        },
        "date": 1786090778999,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 732415.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1180061.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 766855.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 763813.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor                \\nCores: 4"
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
          "id": "f1fab1851731a56919ca3ade95ffca37f3ea8d01",
          "message": "build(deps): bump Jimver/cuda-toolkit from 0.2.35 to 0.2.36\n\nBumps [Jimver/cuda-toolkit](https://github.com/jimver/cuda-toolkit) from 0.2.35 to 0.2.36.\n- [Release notes](https://github.com/jimver/cuda-toolkit/releases)\n- [Commits](https://github.com/jimver/cuda-toolkit/compare/v0.2.35...v0.2.36)\n\n---\nupdated-dependencies:\n- dependency-name: Jimver/cuda-toolkit\n  dependency-version: 0.2.36\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-08-08T08:35:42+08:00",
          "tree_id": "e00d185b3a3bebca6b8e9c131180275191f5fc28",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1fab1851731a56919ca3ade95ffca37f3ea8d01"
        },
        "date": 1786149841764,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 813890.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172942,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 766668.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 1048152.5,
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
          "id": "e9c3c21619ce17f2552afeae2398ef33fbce4ee4",
          "message": "Merge pull request #260 from LoveDaisy/fix/closedform-tolerance-residuals\n\nfix(core): remove the closed-form pyramid's unit-of-measure assumption from its tolerances",
          "timestamp": "2026-08-08T12:02:24+08:00",
          "tree_id": "9f892f4c5b6e1fef411a3819fa50d9ab8f4ccfe7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e9c3c21619ce17f2552afeae2398ef33fbce4ee4"
        },
        "date": 1786162366336,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1098162.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1161755.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 761994.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 709108,
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
          "id": "e9f15c07a86a7eb9878e5a43e149b5137e846df3",
          "message": "Merge pull request #261 from LoveDaisy/feat/gui-test-suite-rebuild\n\ntest(gui): rebuild the GUI test suite into three layers, and gate the cascade defect family",
          "timestamp": "2026-08-10T22:24:25+08:00",
          "tree_id": "e9887d79b9e35fe3454a3514c13308ca2f68fc03",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e9f15c07a86a7eb9878e5a43e149b5137e846df3"
        },
        "date": 1786372558559,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 812047,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1171466,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 720966.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 617102,
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
          "id": "5c2b1a9cd1daae6d11f8b670faca311a48723059",
          "message": "Merge pull request #262 from LoveDaisy/fix/gui-blocked-production-defects\n\nfix(gui): four defects around blank filter rows, slider drags and failed loads",
          "timestamp": "2026-08-11T11:22:23+08:00",
          "tree_id": "86c4753b809868150c88ccfedc5bd8d90ea97394",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c2b1a9cd1daae6d11f8b670faca311a48723059"
        },
        "date": 1786419210614,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1082693.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1166356.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1305831.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V45 96-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 597369.3,
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
          "id": "6ee81825abca90286b1eaef923f6aeaaaabae056",
          "message": "Merge pull request #202 from LoveDaisy/feat/filter-form-big-or\n\nfeat: 放开 filter OR-clause 上限 8/16→4096(纯过滤,染色 mask 不动)",
          "timestamp": "2026-07-16T07:43:52+08:00",
          "tree_id": "c02e2eebc9625ec984d3d85fbb779cf8f25ea060",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6ee81825abca90286b1eaef923f6aeaaaabae056"
        },
        "date": 1784159463852,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
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
          "id": "ec1879077f613c9bbfaabb3079281ef5164bb2de",
          "message": "Merge pull request #203 from LoveDaisy/chore/reconciler-gate-wake-helper\n\nchore: harden reconciler include boundary + dedup wake path (scrum-353 T2 follow-up)",
          "timestamp": "2026-07-16T08:44:05+08:00",
          "tree_id": "b12a3a7a06d99002a3d740ef5c655ebef8581689",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ec1879077f613c9bbfaabb3079281ef5164bb2de"
        },
        "date": 1784163008246,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.4,
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
          "id": "c89c178f203a9956ee50c8af21c7f0300615d053",
          "message": "Merge pull request #204 from LoveDaisy/feat/color-degrade-gui-surfacing\n\nfeat(color-degrade-gui-surfacing): surface all 3 GPU color-degrade caps to GUI modal",
          "timestamp": "2026-07-16T11:53:31+08:00",
          "tree_id": "ac0cf27c253b945d4191e4e914f50f1ad145a0fe",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c89c178f203a9956ee50c8af21c7f0300615d053"
        },
        "date": 1784174452525,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.5,
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
          "id": "ab7aa0b509b61ae45afbf6b3f6fb7846507c53b1",
          "message": "Merge pull request #205 from LoveDaisy/chore/policy-gates\n\nchore(policy-gates): fix the bench compile rot and gate working-note references",
          "timestamp": "2026-07-17T07:52:34+08:00",
          "tree_id": "c40692428310e7ecfc1ae1acafbfeb140902be36",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ab7aa0b509b61ae45afbf6b3f6fb7846507c53b1"
        },
        "date": 1784246401143,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95,
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
          "id": "a610dbde93b0957805c532933a5bde34fabc21a2",
          "message": "Merge pull request #206 from LoveDaisy/fix/degenerate-geometry\n\nfix(core): random face_distance SIGSEGV — scale-relative vertex dedup + non-manifold rejection",
          "timestamp": "2026-07-17T15:08:47+08:00",
          "tree_id": "771c0d3b33012a43a90f8696bf92c5510957945e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a610dbde93b0957805c532933a5bde34fabc21a2"
        },
        "date": 1784272570525,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.1,
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
          "id": "489bfb6288808f55579d34cc8361b00fd84d8fb0",
          "message": "Merge pull request #207 from LoveDaisy/docs/geom-clock-and-benchmark-caveats\n\ndocs: correct two measurement caveats found while calibrating the geometry clock",
          "timestamp": "2026-07-17T18:00:16+08:00",
          "tree_id": "56bd0351ee4a8d7916c831f46b0b8cc6b5ce0f59",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/489bfb6288808f55579d34cc8361b00fd84d8fb0"
        },
        "date": 1784282852801,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79,
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
            "value": 76.1,
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
          "id": "e22f9eea8e5d88a52b44452e88488b73c5bc9ec0",
          "message": "Merge pull request #208 from LoveDaisy/fix/pyramid-geometry-crash-metal\n\nfix(core): pyramid + random face_distance Metal SIGSEGV (count/stride decouple)",
          "timestamp": "2026-07-18T12:55:41+08:00",
          "tree_id": "4664a91a1f187e7cb56fae546dfaa87d149d926c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e22f9eea8e5d88a52b44452e88488b73c5bc9ec0"
        },
        "date": 1784350992931,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
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
          "id": "f29881b3f101b6c06279e10954d789f5984d27bb",
          "message": "Merge pull request #210 from LoveDaisy/chore/gbk-locale-parity-test-unicode\n\nfix(test): ASCII-ize parity test messages for GBK-locale Windows",
          "timestamp": "2026-07-19T09:17:00+08:00",
          "tree_id": "ec1b7089cc6b1d84ac772796a218885ef9dfbe58",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f29881b3f101b6c06279e10954d789f5984d27bb"
        },
        "date": 1784424301076,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.7,
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
          "id": "abbef1227936e40b1753817142344886fae0de78",
          "message": "Merge pull request #209 from LoveDaisy/fix/cuda-unfreeze-geometry-randomization\n\nfix(cuda): unfreeze crystal-shape geometry randomization end-to-end",
          "timestamp": "2026-07-19T09:16:57+08:00",
          "tree_id": "836469c0bb3baf78eec16a14d2eb5e42760849c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/abbef1227936e40b1753817142344886fae0de78"
        },
        "date": 1784425811784,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.3,
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
          "id": "d9d7ee2ff6accd887235198435abfedf87e4f1b2",
          "message": "Merge pull request #212 from LoveDaisy/chore/fix-base\n\nChore/fix base",
          "timestamp": "2026-07-19T10:15:27+08:00",
          "tree_id": "63532f0f8cbe507abecaa438c81d729aee904549",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d9d7ee2ff6accd887235198435abfedf87e4f1b2"
        },
        "date": 1784427803149,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.3,
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
          "id": "163ff642d86a80c43b8197a2ba57a5016cd6d6a7",
          "message": "Merge pull request #213 from LoveDaisy/feat/geometry-pool-and-topology-reuse\n\nfeat(geometry-perf): per-ray K-shape pool on both GPU backends + geometry representation diagnosis",
          "timestamp": "2026-07-20T11:37:25+08:00",
          "tree_id": "099175e5599952910c54ce6d8ff31717f756c00f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/163ff642d86a80c43b8197a2ba57a5016cd6d6a7"
        },
        "date": 1784519100054,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.5,
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
          "id": "4057ce13d077863ae1027d799f1fe8d4c8fe1cb0",
          "message": "Merge pull request #214 from LoveDaisy/feat/geometry-closed-form-representation\n\nfeat(geometry): closed-form hex crystal representation (scrum-386)",
          "timestamp": "2026-07-21T13:25:10+08:00",
          "tree_id": "03e5310167743bc37a0d3c76b1b4600e8b4983c0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4057ce13d077863ae1027d799f1fe8d4c8fe1cb0"
        },
        "date": 1784611975950,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.1,
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
          "id": "0244b5f169af36f52d1fbfcf83ea3ca4df809f12",
          "message": "Merge pull request #215 from LoveDaisy/feat/geometry-exact-domain-audit\n\ngeometry exactness: symbolic-a1 exact oracle (drop __int128) + pyramid apex bug fix + 3-platform verify",
          "timestamp": "2026-07-22T15:28:14+08:00",
          "tree_id": "455c7b6b9c07b5fffdb7b5cc4cd6c4c0844c0eb7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/0244b5f169af36f52d1fbfcf83ea3ca4df809f12"
        },
        "date": 1784705729591,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 72.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.5,
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
          "id": "7bd2f401246d724694c7c02bbce7a37b093f43d4",
          "message": "Merge pull request #216 from LoveDaisy/feat/pyramid-oracle-contract-tests\n\ngeometry test: retire symbolic-α pyramid oracle for three contract-aligned tests",
          "timestamp": "2026-07-22T17:35:30+08:00",
          "tree_id": "316a03a93602e9c23a00832d413abf5c51648733",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7bd2f401246d724694c7c02bbce7a37b093f43d4"
        },
        "date": 1784713399375,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.4,
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
          "id": "de7b4571c336a6d6f4814657c8822c6f149fb091",
          "message": "Merge pull request #217 from LoveDaisy/feat/geom-pool-metal-landing\n\nfeat(geom-pool): wire K-shape pool geom_clock into config + Metal/CUDA backends",
          "timestamp": "2026-07-22T23:04:47+08:00",
          "tree_id": "69bd58055ec2bb19e867aaf6a9ac66a07a16cb55",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de7b4571c336a6d6f4814657c8822c6f149fb091"
        },
        "date": 1784733126624,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 74.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.3,
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
          "id": "c527f60f46b67a4c4bb8bab34f73281efb072b65",
          "message": "Merge pull request #218 from LoveDaisy/feat/cuda-degenerate-geometry-parity\n\nfeat(cuda): degenerate K-shape pool parity + crystal-count assertion (scrum-392)",
          "timestamp": "2026-07-23T12:17:55+08:00",
          "tree_id": "0127101471b6d7dd59e5faea4bc8329d73e60df3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c527f60f46b67a4c4bb8bab34f73281efb072b65"
        },
        "date": 1784780746105,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.6,
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
          "id": "86b954890e231d91593eefe4a83ec335fe440227",
          "message": "Merge pull request #219 from LoveDaisy/feat/crystal-consumption-detriangulation\n\nfeat(core): detriangulate crystal consumption — polygon-granularity incidence sampling, remove triangle mesh from hot path",
          "timestamp": "2026-07-24T08:25:14+08:00",
          "tree_id": "7b83d1927fad720bef607c05a2de71ac1331ef3a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86b954890e231d91593eefe4a83ec335fe440227"
        },
        "date": 1784853155154,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 69,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
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
          "id": "1459b4b094c37b2dd13657b05447d4a342cb6d1b",
          "message": "Merge pull request #220 from LoveDaisy/feat/strong-randomization-downstream-contracts\n\nfix: strong-randomization downstream contracts (filter/render/consumer)",
          "timestamp": "2026-07-24T18:21:13+08:00",
          "tree_id": "418f71e6a27d0ed077d6af076ab0f1ef0f95ab01",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1459b4b094c37b2dd13657b05447d4a342cb6d1b"
        },
        "date": 1784888928118,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 87.3,
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
          "id": "85e12c4e4ae37fe7ffb660b9e22cce0eb96f71c3",
          "message": "Merge pull request #221 from LoveDaisy/feat/core-distribution-cleanup\n\nfeat: crystal shape randomization in GUI + first-class LUMICE_Distribution (BREAKING v4.10)",
          "timestamp": "2026-07-24T20:13:16+08:00",
          "tree_id": "32f4fc909d8d071013c9f3cfd519a135f6cfca2d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85e12c4e4ae37fe7ffb660b9e22cce0eb96f71c3"
        },
        "date": 1784895680482,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.4,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91,
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
          "id": "fa613fb092061df691f456e4ee68497590efe136",
          "message": "Merge pull request #223 from LoveDaisy/feat/gui-shape-randomization-property-table\n\nfeat(gui): crystal shape randomization as a single uniform property table",
          "timestamp": "2026-07-25T09:15:06+08:00",
          "tree_id": "e97db9aed21e5466cd569d61f8ad02416099e8ee",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fa613fb092061df691f456e4ee68497590efe136"
        },
        "date": 1784942604996,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 90.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.8,
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
          "id": "86adcd8f5fc57d211e201db64fb8d77f4bf37f88",
          "message": "Merge pull request #224 from LoveDaisy/feat/capi-scene-opaque-handle\n\nrefactor(c_api): LUMICE_Config value struct → LUMICE_Scene opaque handle (BREAKING v4.12)",
          "timestamp": "2026-07-25T21:38:42+08:00",
          "tree_id": "c1c944a06dc5cd04ab420b22aae79f66f9a92ced",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86adcd8f5fc57d211e201db64fb8d77f4bf37f88"
        },
        "date": 1784987166170,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.5,
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
          "id": "62ba3d6ba31990e43588d1b8c222716cc1a70d5d",
          "message": "Merge pull request #225 from LoveDaisy/feat/gui-visual-regression-coverage\n\ntest(gui): reference-image pixel regression for lens projections + modal layouts",
          "timestamp": "2026-07-25T22:04:33+08:00",
          "tree_id": "750dc3e4cce3891a10ee627c0a5fde4efbbc3a21",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/62ba3d6ba31990e43588d1b8c222716cc1a70d5d"
        },
        "date": 1784988682535,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.3,
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
          "id": "55bfc00af291844e9883a3a68e4c7ac4bc9a8bb3",
          "message": "chore(deps): bump actions/setup-python from 6 to 7\n\nBumps [actions/setup-python](https://github.com/actions/setup-python) from 6 to 7.\n- [Release notes](https://github.com/actions/setup-python/releases)\n- [Commits](https://github.com/actions/setup-python/compare/v6...v7)\n\n---\nupdated-dependencies:\n- dependency-name: actions/setup-python\n  dependency-version: '7'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-07-25T22:18:22+08:00",
          "tree_id": "f42b07d44e062a9971d1edbbc7b7be647d058b05",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/55bfc00af291844e9883a3a68e4c7ac4bc9a8bb3"
        },
        "date": 1784989590328,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.6,
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
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "28e5a9de538df35ed7b5b980592cd051c1109e71",
          "message": "Merge pull request #226 from LoveDaisy/chore/dead-weight-closeout\n\nchore(dead-weight-closeout): delete orphaned Server::CommitConfigFromFile, fix stale ExitRayRecord size comments",
          "timestamp": "2026-07-25T23:16:52+08:00",
          "tree_id": "6622931ec23b0c2436e0728a5bdcc31f2e48b31d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/28e5a9de538df35ed7b5b980592cd051c1109e71"
        },
        "date": 1784993018004,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.4,
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
            "value": 87.6,
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
          "id": "b8248749346e73a9c28c6d9b8c8e68d688f477bc",
          "message": "Merge pull request #227 from LoveDaisy/chore/unified-logging-gate\n\nchore: route src/ diagnostics through the logger, gate bare prints",
          "timestamp": "2026-07-26T00:33:32+08:00",
          "tree_id": "778d66a5d877fafaa592ec918b1102342dcec321",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b8248749346e73a9c28c6d9b8c8e68d688f477bc"
        },
        "date": 1784997691522,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 91.9,
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
          "id": "9b530515528d848ef297c39f68be7d2b0db36841",
          "message": "Merge pull request #228 from LoveDaisy/feat/face-distance-sync-groups\n\nfeat: shape-scalar sync groups (symmetry-preserving shape randomization)",
          "timestamp": "2026-07-27T14:08:08+08:00",
          "tree_id": "e8c7e3774d16d984526ed00e175883c3166fbb79",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9b530515528d848ef297c39f68be7d2b0db36841"
        },
        "date": 1785133054035,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.4,
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
          "id": "1104a952b08241bd61e45bcf7380439a761cbbf7",
          "message": "Merge pull request #229 from LoveDaisy/feat/crystal-sample-count-semantics\n\nfix(stats): make crystals=N a scene property instead of a schedule artifact",
          "timestamp": "2026-07-27T15:17:09+08:00",
          "tree_id": "330fc333a3c88c2f6fcf80ba8e69d53bbf64b031",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/1104a952b08241bd61e45bcf7380439a761cbbf7"
        },
        "date": 1785137107464,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.8,
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
          "id": "ce07b6aea91996dafbf320883cdcd408d861b672",
          "message": "Merge pull request #230 from LoveDaisy/feat/shape-schema-key-single-source\n\nrefactor: give the crystal shape/axis JSON keys one owner in core",
          "timestamp": "2026-07-27T17:39:30+08:00",
          "tree_id": "e1b5c4d911db4ddbe74f30e0e534b9eed7be52b1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ce07b6aea91996dafbf320883cdcd408d861b672"
        },
        "date": 1785145673815,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 90.9,
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
          "id": "19ff683644d3f8e69afc7f9c3226be0b5971d9b1",
          "message": "Merge pull request #231 from LoveDaisy/feat/user-defaults\n\nfeat(gui): user-level defaults layer — generated diff panel + editable axis preset library",
          "timestamp": "2026-07-28T09:03:52+08:00",
          "tree_id": "705f62fb463dfc4fa6d35c7c57aa2735e68ff882",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/19ff683644d3f8e69afc7f9c3226be0b5971d9b1"
        },
        "date": 1785201086043,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 97.1,
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
          "id": "a9d95c29cf1c41e15ecd018702019bef1d7f8f5d",
          "message": "Merge pull request #232 from LoveDaisy/feat/orientation-sample-count-stat\n\nfeat(stats): report orientation sample count as an independent statistic",
          "timestamp": "2026-07-28T13:40:08+08:00",
          "tree_id": "eac8e9336b7adffc48a245cc5cf3dc47ddc6764c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a9d95c29cf1c41e15ecd018702019bef1d7f8f5d"
        },
        "date": 1785217726926,
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
            "value": 96,
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
          "id": "c8492857cbb930e8415baf8267aaadf2082f7b6e",
          "message": "Merge pull request #233 from LoveDaisy/feat/local-test-scope-and-docs\n\nbuild: per-flavor build/install trees + gate bare pytest to the fast subset",
          "timestamp": "2026-07-29T09:07:07+08:00",
          "tree_id": "7796a30d82f61ff9812359a3848ac7bbeb8ccc0a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c8492857cbb930e8415baf8267aaadf2082f7b6e"
        },
        "date": 1785287718718,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.5,
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
          "id": "9955084a05d75819f74dff651c10a063ef2f5b65",
          "message": "Merge pull request #234 from LoveDaisy/feat/panel-settings-editor\n\nfeat(gui): make the defaults panel a pure editor with one source of constraint truth",
          "timestamp": "2026-07-29T09:36:30+08:00",
          "tree_id": "8a4a79fac35d414f9774ddd237a09eb155ad5203",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9955084a05d75819f74dff651c10a063ef2f5b65"
        },
        "date": 1785289534321,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.4,
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
          "id": "92cc54c6635968e5797ada9b806b5f74ab0a8e93",
          "message": "Merge pull request #235 from LoveDaisy/fix/install-hooks-worktree\n\nfix(install-hooks): resolve the hooks dir via git rev-parse --git-path",
          "timestamp": "2026-07-29T09:53:29+08:00",
          "tree_id": "ef5b6fe4f9f35d8c7d6032a1ad0e8c8fbc9a64df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/92cc54c6635968e5797ada9b806b5f74ab0a8e93"
        },
        "date": 1785290511888,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.3,
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
          "id": "95db17f1f62658518c88cdfd144891820be9a119",
          "message": "Merge pull request #236 from LoveDaisy/feat/pytest-invocation-gate\n\nfeat(policy): gate pytest invocations that addopts would silently empty",
          "timestamp": "2026-07-29T15:09:35+08:00",
          "tree_id": "bec8487e460ac1e3d8b09a45c4c9728f0be2162c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/95db17f1f62658518c88cdfd144891820be9a119"
        },
        "date": 1785309507253,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
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
          "id": "86691677ff1522ce09436c46e138a863287d0e4c",
          "message": "Merge pull request #237 from LoveDaisy/feat/gui-sampling-density-stats\n\nfeat(gui): show sampling density (crystal/orientation draws) in the status bar",
          "timestamp": "2026-07-31T18:33:07+08:00",
          "tree_id": "d0ad8ee1d04f228ef63bda3518838ad181adf014",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86691677ff1522ce09436c46e138a863287d0e4c"
        },
        "date": 1785494484546,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 74.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.1,
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
          "id": "04a9998e115ab37676bbe9d09714b1f2240f8350",
          "message": "Merge pull request #238 from LoveDaisy/chore/regen-auto-ev-thresholds\n\nchore(gui-test): 重标定 auto_ev 组 PSNR 阈值（Phase B，N=10）",
          "timestamp": "2026-07-31T20:52:33+08:00",
          "tree_id": "0ea3eb7d2abfc5074d764d0f7d9048e62fa5462d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/04a9998e115ab37676bbe9d09714b1f2240f8350"
        },
        "date": 1785502893405,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.9,
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
          "id": "cc8b11cdb933a3180a7d63f7dba5e9dd11a23514",
          "message": "Merge pull request #239 from LoveDaisy/fix/gui-test-realtiming-load-robustness\n\nfix(gui-test): save_open 收敛判据改为累积光线数，消除负载相关假红",
          "timestamp": "2026-08-01T07:38:52+08:00",
          "tree_id": "0daefbb9d40a4a60972bf996f79b13512022e0c5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cc8b11cdb933a3180a7d63f7dba5e9dd11a23514"
        },
        "date": 1785541715978,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 76.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.5,
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
          "id": "a0fd8622615b2e6f6a5ad0546e09159238614d28",
          "message": "Merge pull request #240 from LoveDaisy/fix/gui-completed-preview-quality-gate\n\nfix(gui): COMPLETED 代终帧绕过质量闸强制上屏（修有限低光线仿真预览永不出图）",
          "timestamp": "2026-08-01T12:52:25+08:00",
          "tree_id": "1c5c8c2e72f475e8aeff98efcea620a5f726d0dd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/a0fd8622615b2e6f6a5ad0546e09159238614d28"
        },
        "date": 1785560482990,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
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
          "id": "f88bf70a41a987c64f3e36ce9f8ff19d102eedac",
          "message": "Merge pull request #243 from LoveDaisy/feat/gui-test-layer-cleanup\n\ntest(gui): gui_test 分层清理 —— 263 个用例迁入无窗口 gui_unit_test，首次获得 CI 覆盖",
          "timestamp": "2026-08-03T07:42:49+08:00",
          "tree_id": "efed823ff3c319c41d56da58dbf9d688687056a8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f88bf70a41a987c64f3e36ce9f8ff19d102eedac"
        },
        "date": 1785714775051,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.7,
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
          "id": "7b8ca898ed0048b18607fdeb7c2c54890cdba249",
          "message": "Merge pull request #244 from LoveDaisy/fix/server-poller-state-reset-ownership\n\nfix(gui): ServerPoller 状态复位收敛为单一 owner + 修重启后状态栏上屏陈旧统计",
          "timestamp": "2026-08-03T11:30:30+08:00",
          "tree_id": "f9d0de83308607c4a960a1b83fe8e645030f244d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7b8ca898ed0048b18607fdeb7c2c54890cdba249"
        },
        "date": 1785728390590,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.5,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
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
          "id": "c3b0c1508ca8975d18284ddfb9089722b087de4b",
          "message": "Merge pull request #245 from LoveDaisy/chore/crash-sentinel-diagnostics\n\nchore: 崩溃哨兵设施补两处诊断缺口（挂起检出 + 两臂 build 日志留痕）",
          "timestamp": "2026-08-04T07:12:15+08:00",
          "tree_id": "7f5982b56736b84800aec2024c45b53a4d40aff5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c3b0c1508ca8975d18284ddfb9089722b087de4b"
        },
        "date": 1785799328693,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "Ubuntu ARM64",
            "value": 99.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.4,
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
          "id": "cf4fdfa21a63e5dc8f790812428e526294c91dcb",
          "message": "Merge pull request #246 from LoveDaisy/investigate/gui-payload-epoch-carryover\n\nfix(gui): 纹理 payload 只在内容确属当前世代时才物化",
          "timestamp": "2026-08-04T07:30:24+08:00",
          "tree_id": "309471c18a2233f37de0d05fd81efcbb05275e73",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/cf4fdfa21a63e5dc8f790812428e526294c91dcb"
        },
        "date": 1785800451480,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.1,
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
          "id": "84faab640e80563e1772399f83d2cc8c5da23afa",
          "message": "Merge pull request #247 from LoveDaisy/fix/gui-display-time-stale-payload-publish\n\nfix(gui-test): 序列化被唤醒的全局 poller，消除 display-time 编辑后的撕裂快照",
          "timestamp": "2026-08-04T10:06:23+08:00",
          "tree_id": "ba9ee6829ca13f0bb812b3833a070ae347c4fae9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/84faab640e80563e1772399f83d2cc8c5da23afa"
        },
        "date": 1785809765385,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 74,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.7,
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
          "id": "fb721d5b41ec6b834ab8f92634398f5a67bb73c2",
          "message": "Merge pull request #248 from LoveDaisy/explore/visual-regression-layer-value\n\ntest(gui): 退役 auto_ev 视觉回归组，把这一层接进 CI 并给「红了怎么办」写下判据",
          "timestamp": "2026-08-04T21:05:45+08:00",
          "tree_id": "07403f81e4fa76b1b60fe18689fd37d91fec211f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fb721d5b41ec6b834ab8f92634398f5a67bb73c2"
        },
        "date": 1785849360452,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 94.9,
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
          "id": "6e95f5c4505cac81d3efb2fbfec80b26926ff77c",
          "message": "Merge pull request #249 from LoveDaisy/explore/gui-test-suite-from-scratch\n\ndocs(testing-architecture): 新增 §4.8 —— GUI 套件形状的机制层诊断与工作规则",
          "timestamp": "2026-08-05T00:54:42+08:00",
          "tree_id": "d211f764eb7e4218ae46f8a34134e89f34aa0e31",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6e95f5c4505cac81d3efb2fbfec80b26926ff77c"
        },
        "date": 1785863089218,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.3,
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
          "id": "14885a4009086138c80492f57dd454c3379f93a2",
          "message": "Merge pull request #250 from LoveDaisy/feat/capi-result-lifetime-ownership\n\nrefactor(capi): 结果数据改为不可变引用计数帧 + 不透明句柄，净删六个旧 getter",
          "timestamp": "2026-08-05T10:49:05+08:00",
          "tree_id": "7d8fafa792635f949c3b78d1b503f263d2c404cc",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/14885a4009086138c80492f57dd454c3379f93a2"
        },
        "date": 1785898754797,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.6,
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
          "id": "b21fe562ba8cdf4e0cf66f9007ee180481bfadcd",
          "message": "Merge pull request #251 from LoveDaisy/feat/preview-lifecycle-invariant-closure\n\nGUI 预览生命周期不变量族级收口（scrum-429）：I3/I4 补齐 + 新增 I7 完成蕴含排空",
          "timestamp": "2026-08-06T10:26:27+08:00",
          "tree_id": "67e60db6fdf70c87913882cb11a32d89376d886c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b21fe562ba8cdf4e0cf66f9007ee180481bfadcd"
        },
        "date": 1785983775005,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 88.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.6,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.2,
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
          "id": "b5d4402f2d5824a55e160f6cffa7e731746c0108",
          "message": "Merge pull request #252 from LoveDaisy/chore/micro-debt-sweep\n\nchore: 微债一次结清（帧 RAII 收敛 / GUI 日志装配 / 可移植测试路径 / C API 边界证据）",
          "timestamp": "2026-08-06T12:31:23+08:00",
          "tree_id": "03aad82a20199d682144c4ce75301214e26541c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b5d4402f2d5824a55e160f6cffa7e731746c0108"
        },
        "date": 1785991307285,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 73.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.3,
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
          "id": "5a8f8982aa3d6bf634f3f13b6cf53ccc29bce9a4",
          "message": "Merge pull request #253 from LoveDaisy/refactor/user-defaults-write-surface-closure\n\nrefactor(gui): close the parallel user-defaults write surface (434)",
          "timestamp": "2026-08-06T16:41:53+08:00",
          "tree_id": "e39098a9bc1cf37fc4ce2c511b3cc4c9e248608f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5a8f8982aa3d6bf634f3f13b6cf53ccc29bce9a4"
        },
        "date": 1786006351618,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 94.1,
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
          "id": "4e3f2bf4a1010ee5fe92f5a0cb24a863a3d96272",
          "message": "Merge pull request #254 from LoveDaisy/fix/revert-field-scope-alignment\n\nfix(gui): align Revert's field scope with the predicate that decides what counts as a change",
          "timestamp": "2026-08-06T17:07:01+08:00",
          "tree_id": "39a256ded94aa63c7c11aaeb8f2c923af34401c7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/4e3f2bf4a1010ee5fe92f5a0cb24a863a3d96272"
        },
        "date": 1786007859677,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.1,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.2,
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
          "id": "559b976cfc69c740bfbb7bcf1a4c9f0bac592e3c",
          "message": "Merge pull request #255 from LoveDaisy/feat/gui-rules-as-data\n\nMake three GUI rules queryable data, and replace the grid tests they forced",
          "timestamp": "2026-08-07T07:52:53+08:00",
          "tree_id": "e263eeb0da176a674de554034f5744e15cc45bf6",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/559b976cfc69c740bfbb7bcf1a4c9f0bac592e3c"
        },
        "date": 1786060889733,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.6,
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
          "id": "985357aace8cb86e666f1c4e317f08c847f2af51",
          "message": "Merge pull request #256 from LoveDaisy/fix/pyramid-closed-form-geometry-defects\n\nfix(core): close the closed-form pyramid's structural geometry defects",
          "timestamp": "2026-08-07T14:54:11+08:00",
          "tree_id": "08e78fc2e34f33e8a8b828d75ac37a0294dd0cad",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/985357aace8cb86e666f1c4e317f08c847f2af51"
        },
        "date": 1786086324448,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.2,
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
          "id": "6b3c6b4ca3743e19c327b64957e7c30aea188970",
          "message": "Merge pull request #257 from LoveDaisy/feat/config-default-semantics\n\nMake core's implicit config defaults into written contracts (prob / axis type / absent axis)",
          "timestamp": "2026-08-07T15:41:10+08:00",
          "tree_id": "8426a314fa4cb0aff7bfe853eb95de84ef4b10f0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6b3c6b4ca3743e19c327b64957e7c30aea188970"
        },
        "date": 1786089131152,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.9,
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
          "id": "35c7cd3f1734523f8812d509152a6b3ff8ab2ea7",
          "message": "Merge pull request #258 from LoveDaisy/fix/preview-drag-gain-fov\n\nfix(gui): scale preview drag by the lens's angular resolution",
          "timestamp": "2026-08-07T16:10:31+08:00",
          "tree_id": "91628e3757321fdf8723aac188f3c9c5be6aa3e5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/35c7cd3f1734523f8812d509152a6b3ff8ab2ea7"
        },
        "date": 1786090781827,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 96.5,
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
          "id": "f1fab1851731a56919ca3ade95ffca37f3ea8d01",
          "message": "build(deps): bump Jimver/cuda-toolkit from 0.2.35 to 0.2.36\n\nBumps [Jimver/cuda-toolkit](https://github.com/jimver/cuda-toolkit) from 0.2.35 to 0.2.36.\n- [Release notes](https://github.com/jimver/cuda-toolkit/releases)\n- [Commits](https://github.com/jimver/cuda-toolkit/compare/v0.2.35...v0.2.36)\n\n---\nupdated-dependencies:\n- dependency-name: Jimver/cuda-toolkit\n  dependency-version: 0.2.36\n  dependency-type: direct:production\n  update-type: version-update:semver-patch\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-08-08T08:35:42+08:00",
          "tree_id": "e00d185b3a3bebca6b8e9c131180275191f5fc28",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f1fab1851731a56919ca3ade95ffca37f3ea8d01"
        },
        "date": 1786149844097,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.7,
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
          "id": "e9c3c21619ce17f2552afeae2398ef33fbce4ee4",
          "message": "Merge pull request #260 from LoveDaisy/fix/closedform-tolerance-residuals\n\nfix(core): remove the closed-form pyramid's unit-of-measure assumption from its tolerances",
          "timestamp": "2026-08-08T12:02:24+08:00",
          "tree_id": "9f892f4c5b6e1fef411a3819fa50d9ab8f4ccfe7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e9c3c21619ce17f2552afeae2398ef33fbce4ee4"
        },
        "date": 1786162368321,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.1,
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
          "id": "e9f15c07a86a7eb9878e5a43e149b5137e846df3",
          "message": "Merge pull request #261 from LoveDaisy/feat/gui-test-suite-rebuild\n\ntest(gui): rebuild the GUI test suite into three layers, and gate the cascade defect family",
          "timestamp": "2026-08-10T22:24:25+08:00",
          "tree_id": "e9887d79b9e35fe3454a3514c13308ca2f68fc03",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e9f15c07a86a7eb9878e5a43e149b5137e846df3"
        },
        "date": 1786372560563,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95,
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
          "id": "5c2b1a9cd1daae6d11f8b670faca311a48723059",
          "message": "Merge pull request #262 from LoveDaisy/fix/gui-blocked-production-defects\n\nfix(gui): four defects around blank filter rows, slider drags and failed loads",
          "timestamp": "2026-08-11T11:22:23+08:00",
          "tree_id": "86c4753b809868150c88ccfedc5bd8d90ea97394",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c2b1a9cd1daae6d11f8b670faca311a48723059"
        },
        "date": 1786419212572,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.8,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 98.3,
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
      }
    ]
  }
}