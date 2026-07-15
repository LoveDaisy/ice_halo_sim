window.BENCHMARK_DATA = {
  "lastUpdate": 1784074236589,
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7fb0d1f4478fc0f63a7a5e2c5fbde2d6a7e073d2",
          "message": "Merge pull request #145 from LoveDaisy/chore/cleanup-metal-compile-review-minors\n\nchore: clean up Metal compile-path review minors (task-282/283 衍生, #293)",
          "timestamp": "2026-06-24T10:28:16+08:00",
          "tree_id": "c31eea8c0d7df06b3064a38d766c0960feaa9fde",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7fb0d1f4478fc0f63a7a5e2c5fbde2d6a7e073d2"
        },
        "date": 1782268464538,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 391346.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 617801.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 393791.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 326091.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2f8eb7bbab1521ea2da2b78755c8afe7e394a804",
          "message": "Merge pull request #146 from LoveDaisy/chore/release-prep\n\nchore: release prep v4.3.1 (perf-doc baseline + TraceCrystalBatch cleanup)",
          "timestamp": "2026-06-24T11:39:57+08:00",
          "tree_id": "3dca5e48d31d31d09f763da6c0d5dcc69e2ac0e7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2f8eb7bbab1521ea2da2b78755c8afe7e394a804"
        },
        "date": 1782272730858,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 311576.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 612812.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 463891.9,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 369139,
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
          "id": "19cbaf77d4bb88b85b6d5ed6f6330d0d19d86966",
          "message": "Merge pull request #147 from LoveDaisy/feat/cuda-backend-mvp\n\nfeat(gpu): CUDA backend MVP — single-MS no-filter raw-XYZ parity (scrum-#295)",
          "timestamp": "2026-06-25T15:24:53+08:00",
          "tree_id": "76d63a8f08faefd488e6b75f7425f402972e0d4d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/19cbaf77d4bb88b85b6d5ed6f6330d0d19d86966"
        },
        "date": 1782372669049,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 391974.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 619084.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 377209.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 325421.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5c9c62a9719c8db0cd540c81261e3b3d55b6f615",
          "message": "Merge pull request #148 from LoveDaisy/feat/cuda-backend-complete\n\nfeat(cuda): CUDA backend complete (scrum-296) — Metal 功能对齐 + 吞吐就绪",
          "timestamp": "2026-06-26T20:53:54+08:00",
          "tree_id": "b5aacabd73d8a1a44c91f040ab1ef998e00536d0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c9c62a9719c8db0cd540c81261e3b3d55b6f615"
        },
        "date": 1782478801959,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 447508.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 595805.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396122.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 323909,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5710d6cd13a3290e0aaad7d92b4ac9f2a549a332",
          "message": "Merge pull request #149 from LoveDaisy/worktree-fix-stats-ray-count-u32-overflow\n\nfix(stats): widen ray-count types to 64-bit (Windows u32 overflow) — task-297",
          "timestamp": "2026-06-26T21:15:50+08:00",
          "tree_id": "1fcf076eb29b52ca6f7e11c988fee4e7340dbaa5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5710d6cd13a3290e0aaad7d92b4ac9f2a549a332"
        },
        "date": 1782480105215,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 448961,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 599844.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 387377.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 359670.5,
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
          "id": "df47f8ce94980421ea457cf2a81983343c84732b",
          "message": "Merge pull request #150 from LoveDaisy/chore/deferred-quality-cleanup\n\nchore: deferred quality cleanup (scrum-298) — e2e ref regen + geometry predicate single-source + ray_num float precision",
          "timestamp": "2026-06-26T22:50:03+08:00",
          "tree_id": "638406a85c800d2be16737ae7522efb8951a83fd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/df47f8ce94980421ea457cf2a81983343c84732b"
        },
        "date": 1782485774433,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 355833.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582825.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 398785.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 322388.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7bd4807017bdde3f3cb961988b4ab9991eeec42b",
          "message": "Merge pull request #152 from LoveDaisy/feat/cuda-multi-ci-correctness\n\nfix(gpu): CUDA full multi-CI correctness + device-side recombine shuffle (Metal+CUDA)",
          "timestamp": "2026-06-27T21:44:16+08:00",
          "tree_id": "afa0020517cdb071aab346c7fde6ef574372c0f9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7bd4807017bdde3f3cb961988b4ab9991eeec42b"
        },
        "date": 1782568231708,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 327620.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 586055.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 392319.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 283198.8,
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
          "id": "bc66f2ec2fd37503c13844019d347f39549e4228",
          "message": "Merge pull request #153 from LoveDaisy/feat/gpu-device-fused-accumulation\n\nfeat(scrum-302): device-fused XYZ accumulation (Metal + CUDA)",
          "timestamp": "2026-06-28T10:45:30+08:00",
          "tree_id": "44b7f418587aa18fc23e29305d3eeda8bd8bb3c9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/bc66f2ec2fd37503c13844019d347f39549e4228"
        },
        "date": 1782615123979,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 434397.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 595051.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 395164.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 319399.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c2c5801cbe82410b1423d904c37480dc4ec03185",
          "message": "Merge pull request #154 from LoveDaisy/feat/cuda-async-engine-port\n\nperf(scrum-304): persist CUDA buffers across sessions — CUDA throughput competitive + bench standardized",
          "timestamp": "2026-06-29T09:28:47+08:00",
          "tree_id": "539bc1c7bc7672f023725ee7be21bafaa4fb88d5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c2c5801cbe82410b1423d904c37480dc4ec03185"
        },
        "date": 1782696879030,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 417720.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593159.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 370261.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 341818.1,
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
          "id": "3c13a634c66560579a93d2b87cccd690510b1c83",
          "message": "Merge pull request #155 from LoveDaisy/feat/cuda-async-engine\n\nperf(scrum-306): CUDA throughput 37M→~114M (dispatch default + dead-buffer cap)",
          "timestamp": "2026-06-29T17:22:12+08:00",
          "tree_id": "0bfd23fcfaa5e3614e447a2936ea2d73bc2879af",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3c13a634c66560579a93d2b87cccd690510b1c83"
        },
        "date": 1782725331116,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 380258.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584579.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 383252.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 322089.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "85c35073907dd5ea3eb9e9bf64bdcb20be8d0ac9",
          "message": "Merge pull request #156 from LoveDaisy/fix/randomsample-nomatch-entry-leak\n\nfix(geo3d): RandomSample no-match fallback for MSVC 77H light leak (curr_p==0.0 → entry-face bug)",
          "timestamp": "2026-06-30T14:48:02+08:00",
          "tree_id": "31d7f3bcd82d107a3ecd3e02f1be0e8105ec0277",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85c35073907dd5ea3eb9e9bf64bdcb20be8d0ac9"
        },
        "date": 1782802419989,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 299132.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592485.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 389508.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 319939.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "270ec637801ed3a00639faf6e210e2a2a239c19a",
          "message": "Merge pull request #157 from LoveDaisy/feat/cuda-windows-validation\n\nCUDA on Windows: validation (#309) + delivery cluster (#310)",
          "timestamp": "2026-07-01T09:10:33+08:00",
          "tree_id": "2898159b77ce8341ec472358b0b3160cc9f7d1f2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/270ec637801ed3a00639faf6e210e2a2a239c19a"
        },
        "date": 1782868594719,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 258041.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588257.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 381713.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321705.9,
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
          "id": "9d189a99ead4ae89afa02c1323a666f5ef9105dc",
          "message": "build(deps): bump actions/cache from 5 to 6\n\nBumps [actions/cache](https://github.com/actions/cache) from 5 to 6.\n- [Release notes](https://github.com/actions/cache/releases)\n- [Changelog](https://github.com/actions/cache/blob/main/RELEASES.md)\n- [Commits](https://github.com/actions/cache/compare/v5...v6)\n\n---\nupdated-dependencies:\n- dependency-name: actions/cache\n  dependency-version: '6'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-07-01T10:01:02+08:00",
          "tree_id": "9dfc76647b46f224989c5ce3bbb595c48119842a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9d189a99ead4ae89afa02c1323a666f5ef9105dc"
        },
        "date": 1782871617562,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 288546.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 575329.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 474642.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 322239.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "085d3a1ef63b9ff2eba44ab57ad6b3d40cac33d1",
          "message": "Merge pull request #158 from LoveDaisy/feat/gpu-misc\n\nchore(cleanup): CUDA dead-code + CI Node24 bump + exit-seam crystals stat fix (scrum-311)",
          "timestamp": "2026-07-01T13:10:24+08:00",
          "tree_id": "6524533648ef89053374c3911be4dec2d1722643",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/085d3a1ef63b9ff2eba44ab57ad6b3d40cac33d1"
        },
        "date": 1782882986878,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 323671.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587664.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397971.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 250577,
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
          "id": "f8cdfeb59aa4b54dbb45e9bafbdfec1ea6176396",
          "message": "Merge pull request #159 from LoveDaisy/feat/gpu-readback-third-clock\n\nfeat(gpu): third-clock readback decoupling — fix high-resolution GPU throughput",
          "timestamp": "2026-07-01T21:22:40+08:00",
          "tree_id": "ce2f065d0985c3fc1f1e81c395b8260d8d1498b9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f8cdfeb59aa4b54dbb45e9bafbdfec1ea6176396"
        },
        "date": 1782912535392,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 318135.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584838.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 451950.1,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 321299.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7010f091e1a9f26c946233253304c160df89a095",
          "message": "Merge pull request #160 from LoveDaisy/chore/gpu-doc-consolidation\n\ndocs+bench: GPU doc consolidation + collapse GPU --benchmark to one steady pass",
          "timestamp": "2026-07-02T09:15:42+08:00",
          "tree_id": "ce207ad8fbe6fc8a13cd0d39fd8d4dc784aba9c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7010f091e1a9f26c946233253304c160df89a095"
        },
        "date": 1782955302286,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 437378.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 590131.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 390966.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 325090.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "85fdfef28b6cbde3034622ca889e9d508457ca0d",
          "message": "Merge pull request #162 from LoveDaisy/feat/gpu-projection-parity\n\nfeat(gpu): unify render projection into single source + all 11 projections on Metal/CUDA (scrum-315)",
          "timestamp": "2026-07-02T14:58:05+08:00",
          "tree_id": "94499fa0850c528ab90de71578462a098aae6efb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85fdfef28b6cbde3034622ca889e9d508457ca0d"
        },
        "date": 1782975847096,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 319442.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594028.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 366374.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 309932.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "23f16349a76ae1ea75539389ae8c1beb8c83b93e",
          "message": "Merge pull request #163 from LoveDaisy/ci/parallelize-slow-e2e\n\nci: parallelize slow-e2e with pytest-xdist, isolate throughput gates",
          "timestamp": "2026-07-02T17:42:26+08:00",
          "tree_id": "6097dda8ad93d8a7322ae8086936166b9c7ecbd1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/23f16349a76ae1ea75539389ae8c1beb8c83b93e"
        },
        "date": 1782985715616,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 470510.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 582690.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 590594.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 291213.4,
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
          "id": "ba2b6bbf58cb0df294c2994d9af0c0c39a8fe3d4",
          "message": "Merge pull request #161 from LoveDaisy/feat/gpu-bench-drain-aligned-rate\n\nfix(bench): drain-count-driven GPU --benchmark rate (fixes 5× under-report)",
          "timestamp": "2026-07-02T17:54:37+08:00",
          "tree_id": "5f5489b014b23a7dbed42902f7ac0d35530eb9a9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ba2b6bbf58cb0df294c2994d9af0c0c39a8fe3d4"
        },
        "date": 1782986442835,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 428354.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 606961.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 401232.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 362925.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "941ef7f1f57c56309fd9833e8bc4ecedcdb7914c",
          "message": "Merge pull request #164 from LoveDaisy/feat/gpu-rng-ray-index-uint64\n\nfix(gpu-rng): lift device-gen PCG ray-index 32-bit cap (uint64 lo/hi)",
          "timestamp": "2026-07-03T00:46:46+08:00",
          "tree_id": "b4bda3ea0cc4d5a7add7d4f7445183855fe24710",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/941ef7f1f57c56309fd9833e8bc4ecedcdb7914c"
        },
        "date": 1783011181723,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 356491.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609101.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 402764.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 360884.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c9d7885ae89b30fe1a6ebf168686aa712613d18d",
          "message": "Merge pull request #165 from LoveDaisy/feat/gui-cli-render-alignment\n\nfix(gui): align GUI preview lens orientation with CLI render (scrum-320)",
          "timestamp": "2026-07-03T10:43:42+08:00",
          "tree_id": "690cea094f72fb75ca121307df859bfdf6ea2575",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c9d7885ae89b30fe1a6ebf168686aa712613d18d"
        },
        "date": 1783047007940,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 485317.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 604611.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400813.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 363798.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5b75ab9e83d57e35e755c4898cff59adaaf1faf",
          "message": "Merge pull request #166 from LoveDaisy/feat/azimuth-handedness-alignment\n\nfix(render): unify screen handedness to right=+az (scrum-321)",
          "timestamp": "2026-07-03T16:07:56+08:00",
          "tree_id": "83dad4392b494a6bad816129042c2a0250c83832",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5b75ab9e83d57e35e755c4898cff59adaaf1faf"
        },
        "date": 1783066437720,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 358113,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 605812.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 411337.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 361206.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fd719cc6d151b919a44e8ff1717f6e393a9bb12c",
          "message": "Merge pull request #167 from LoveDaisy/feat/gui-lifecycle-clock-decouple\n\nGUI preview lifecycle: clock-decouple to single-source epoch/lifecycle (I1–I6)",
          "timestamp": "2026-07-03T16:35:42+08:00",
          "tree_id": "6e1aaabdbcb0ad556df9d66266d4156f23f8c2dd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fd719cc6d151b919a44e8ff1717f6e393a9bb12c"
        },
        "date": 1783068101909,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 388778.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 602876.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 387625.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 407142.8,
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
          "id": "938964638e672bf93d079a6380a9a2e136258f18",
          "message": "Merge pull request #168 from LoveDaisy/feat/task-gui-custom-spectrum\n\nfeat(gui): custom discrete spectrum editor + ray_num total-across-wavelengths semantics (task-323)",
          "timestamp": "2026-07-04T00:50:09+08:00",
          "tree_id": "d051e01b6ab81a4e3032c05d1af88fb45e1c9b93",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/938964638e672bf93d079a6380a9a2e136258f18"
        },
        "date": 1783097774850,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 366392.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611789.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 408083.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 330683.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "026928a679c7e511723436c1dd4d8f3ae18c16ab",
          "message": "Merge pull request #169 from LoveDaisy/feat/gui-ms-prob-footguns\n\ngui: MS layer prob footgun guards (four-state slider, +Layer promotion, CLI warning)",
          "timestamp": "2026-07-04T01:22:18+08:00",
          "tree_id": "8ebb7d1b8443f8baf800439ff34a54dfd2e8a09e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/026928a679c7e511723436c1dd4d8f3ae18c16ab"
        },
        "date": 1783099701122,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 402910.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 611864.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 394653.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 354181.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "32962163bd51949dbeed818117d5126fdc0b35f3",
          "message": "Merge pull request #171 from LoveDaisy/feat/near-pole-area-measure-sampling\n\nfix(gpu): root-fix near-pole rejection waste via unified tight-envelope area-measure sampling (scrum-328)",
          "timestamp": "2026-07-04T19:38:44+08:00",
          "tree_id": "471a6c43b8b361b747c4f824aa4b6910c5746a76",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/32962163bd51949dbeed818117d5126fdc0b35f3"
        },
        "date": 1783165503691,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 496912.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 610909.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 401662.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 359790.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "21cba8ceb52eb890be158eb7ae8abf66a3036414",
          "message": "Merge pull request #172 from LoveDaisy/feat/capi-filter-typed-commit\n\nC API filter typed-struct commit convergence (327) + backend-swap preview fix",
          "timestamp": "2026-07-04T21:12:29+08:00",
          "tree_id": "b0c7d73eda939b5e2e3ab1873a5585a6d28b8373",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/21cba8ceb52eb890be158eb7ae8abf66a3036414"
        },
        "date": 1783171117524,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 466044.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 608976,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 404212.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 356450,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "de22a871800c89cf92e502b71a29f91d134fb927",
          "message": "Merge pull request #173 from LoveDaisy/feat/gui-spectrum-modal-reset-button\n\nfeat(gui): Custom Spectrum modal Reset + overlimit warning polish (GUI small-fixes batch)",
          "timestamp": "2026-07-05T12:12:29+08:00",
          "tree_id": "b33e8efd36f103f5d98a7ee3ea26bf4e2008c972",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de22a871800c89cf92e502b71a29f91d134fb927"
        },
        "date": 1783225123484,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 327738,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 609901.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 386720.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 362941.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7c47f07a520ff7130d21cc6c16755c008e6e5c8d",
          "message": "Merge pull request #174 from LoveDaisy/feat/unify-orientation-sampling-cosine-measure\n\nUnify orientation latitude sampling to a cosine-measure inverse-CDF LUT",
          "timestamp": "2026-07-06T08:58:34+08:00",
          "tree_id": "5a7bb8bc6d9e114d130ac42451f7b3b85b9ea5ab",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7c47f07a520ff7130d21cc6c16755c008e6e5c8d"
        },
        "date": 1783299870803,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 492469.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 607841.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 405853.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 400524.7,
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
          "id": "aff094ef773a9d78983dc1d7d1aabf793757c22d",
          "message": "Merge pull request #175 from LoveDaisy/feat/raypath-color-foundation\n\nraypath-color foundation: per-ray component mask across CPU/Metal/CUDA (scrum-331)",
          "timestamp": "2026-07-06T09:26:54+08:00",
          "tree_id": "3cd3bee8f8ae3b0625d56d07253b29d0593b3f46",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/aff094ef773a9d78983dc1d7d1aabf793757c22d"
        },
        "date": 1783301590329,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 349547.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 589084.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 476306.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 342374.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b634945ce767a3531a89893427812f8f4a905607",
          "message": "Merge pull request #176 from LoveDaisy/chore/pre-release\n\nchore: pre-release housekeeping + raypath-color phase-3 blueprint",
          "timestamp": "2026-07-06T11:09:46+08:00",
          "tree_id": "9f67656a300d6ed2c850553edb269e19f5effc58",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b634945ce767a3531a89893427812f8f4a905607"
        },
        "date": 1783307744344,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 317596.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594430.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 405608.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 320057.3,
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
          "id": "fc59c7253b6ae043676a2692ef0b8e55ea060405",
          "message": "Merge pull request #177 from LoveDaisy/feat/filter-editor-uplift\n\nfeat(gui): H5 sum-of-products filter editor + input ergonomics (scrum-333/334)",
          "timestamp": "2026-07-07T01:14:22+08:00",
          "tree_id": "adac9b7620bed310e892d5298d721d05c6f88500",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc59c7253b6ae043676a2692ef0b8e55ea060405"
        },
        "date": 1783358451831,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 293373.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592305.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 411083.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 347735.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b7b53fef279f91c39cc06311da7ba1a5b5f52ceb",
          "message": "Merge pull request #178 from LoveDaisy/perf/latlut-shared-cache\n\nperf(latlut): fix mixed-axis multi-crystal LUT rebuild thrash (~20x)",
          "timestamp": "2026-07-07T09:10:44+08:00",
          "tree_id": "c5f07544b862c9153a6ffd644ad93eba311fa21a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b7b53fef279f91c39cc06311da7ba1a5b5f52ceb"
        },
        "date": 1783387057271,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 363945.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 587405.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 388091.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345113.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3adb02485be2774b81a838d29d3e57f99be9cb2a",
          "message": "Merge pull request #179 from LoveDaisy/fix/crystal-preview-thumbnail\n\nfix(gui): correct crystal-preview face labels + reset pose on card switch (task-337)",
          "timestamp": "2026-07-07T11:36:30+08:00",
          "tree_id": "29de997cfb09809b633bfdc7d7d31c581fc1679d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3adb02485be2774b81a838d29d3e57f99be9cb2a"
        },
        "date": 1783395714362,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 455805.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 597076.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 393457.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 400884.8,
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
          "id": "e5c2ab33efb98c2c0d0100e30facccc50e2782ab",
          "message": "Merge pull request #180 from LoveDaisy/fix/modal-edit-state-leak\n\nfix(gui): stop edit-modal state leaking across crystal entries",
          "timestamp": "2026-07-07T15:41:11+08:00",
          "tree_id": "adc13365d8c0abdaf221ec7ba1ae23d7fae8414f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e5c2ab33efb98c2c0d0100e30facccc50e2782ab"
        },
        "date": 1783410480968,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 318499.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593777.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 406478.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 342359.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8d88f08734051714d5bb893476504dc8be228f72",
          "message": "Merge pull request #181 from LoveDaisy/fix/regen-auto-ev-refs\n\nfix(auto-ev): regen stale visual refs + recalibrate thresholds (kill 31% flake)",
          "timestamp": "2026-07-07T22:15:20+08:00",
          "tree_id": "41512c22064985ece3e71d5e151eba5e56362085",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d88f08734051714d5bb893476504dc8be228f72"
        },
        "date": 1783434135057,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 451083.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 589827.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 396366.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 332044.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "11e8a6b1b736e3f9efad2e54fb068a48b09283d3",
          "message": "Merge pull request #182 from LoveDaisy/feat/color-components\n\nfeat(raypath-color): per-raypath color engine — color-class schema + rule-lane compositor (CLI/core, CPU)",
          "timestamp": "2026-07-08T00:44:52+08:00",
          "tree_id": "7eb8f0d87c2ce98aeec8d87910abae09d1f52567",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/11e8a6b1b736e3f9efad2e54fb068a48b09283d3"
        },
        "date": 1783443123461,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 298233.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 591552.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 474998.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 351320.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "79002e6775a0a895acab77df2ac4a9586a5e2105",
          "message": "Merge pull request #183 from LoveDaisy/perf/gui-test-fixed-dt\n\nperf(gui-test): decouple frame budget from wall-clock (--fixed-dt, 16x faster correctness pool)",
          "timestamp": "2026-07-08T16:52:29+08:00",
          "tree_id": "e4db6edaea0749753c8f12e09e42ae50187e3c48",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/79002e6775a0a895acab77df2ac4a9586a5e2105"
        },
        "date": 1783501105154,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 456876.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 584702.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 399330.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 415537.8,
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
          "id": "6bd70df3d6b1c12a3ad92d53e65d7c71dc5b9c75",
          "message": "Merge pull request #184 from LoveDaisy/feat/raypath-color-design2\n\nfeat(raypath-color): phase-3b Design-2 redirect + GUI color window + preview v1 + dynamic-ABI fix",
          "timestamp": "2026-07-08T23:06:24+08:00",
          "tree_id": "dd27c9f90fa94a0633710927433158a0da5b81df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6bd70df3d6b1c12a3ad92d53e65d7c71dc5b9c75"
        },
        "date": 1783523614569,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 424116.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593111.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 386966,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 370083.8,
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
          "id": "3810ef19d13d584242a467c18b2a4f3236b91c9b",
          "message": "Merge pull request #185 from LoveDaisy/feat/raypath-color-gui-polish\n\nper-raypath 染色 GUI phase-3b polish（scrum-345/346 + task-347）",
          "timestamp": "2026-07-10T08:56:20+08:00",
          "tree_id": "aed23ac51a7a8611ce569fa549c3ffb93fa2d180",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3810ef19d13d584242a467c18b2a4f3236b91c9b"
        },
        "date": 1783645439127,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 451630,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 594471.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 406835.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 333426.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d44e3da99e84bc27666910b9d8bb3c2adb090ef6",
          "message": "Merge pull request #186 from LoveDaisy/feat/raypath-color-gui-polish-2\n\nper-raypath 染色 GUI polish（三轮 on-screen 反馈：状态提示/ergonomics/Open 旧图残留根治）",
          "timestamp": "2026-07-11T07:05:30+08:00",
          "tree_id": "69725b832ee1697cbe2d6864cbe7ecee780b549d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d44e3da99e84bc27666910b9d8bb3c2adb090ef6"
        },
        "date": 1783725207810,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 383651.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 595247.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 387015.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337545.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5e45e77b49c06d5ed9f7ccc5f96770da8e6a371d",
          "message": "Merge pull request #187 from LoveDaisy/feat/gui-state-reconcile\n\nGUI 状态治理专项：统一状态转换范式（explore-352 → scrum-353 + 354/355）",
          "timestamp": "2026-07-12T13:55:17+08:00",
          "tree_id": "a31a72d3fb0389df9c193f47ffdf4a14a1e33b30",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5e45e77b49c06d5ed9f7ccc5f96770da8e6a371d"
        },
        "date": 1783836072107,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 462541.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593753.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 397087.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 413995,
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
          "id": "3ddafb383f6c183c3af704f4b34391509bb2fe09",
          "message": "Merge pull request #188 from LoveDaisy/feat/color-predicate-symmetry\n\nfeat: colour predicate PBD symmetry (scrum-356)",
          "timestamp": "2026-07-12T19:48:34+08:00",
          "tree_id": "839ef5a7d5f888a3d0ea9d623af51b57ecc2caca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3ddafb383f6c183c3af704f4b34391509bb2fe09"
        },
        "date": 1783857377696,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 436433.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 597088.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 399528.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 353368,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5e97f32dfff1df95c079d3e6a6dcf1d31ff2b870",
          "message": "Merge pull request #189 from LoveDaisy/feat/local-cleanup-sweep\n\nchore: local cleanup sweep — popcount gate, sibling-race, filter test, doc fixup (scrum-357)",
          "timestamp": "2026-07-13T00:20:25+08:00",
          "tree_id": "c297e4cca53bbc42294ff1e88146c4bcf2c33696",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5e97f32dfff1df95c079d3e6a6dcf1d31ff2b870"
        },
        "date": 1783873677169,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 449663.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588007.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 390381.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 334274.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "112183c86595221b64fc96eba2f3b6a5ba90d3b4",
          "message": "Merge pull request #190 from LoveDaisy/feat/raypath-color-gpu-parity\n\nphase-3c: GPU 染色三后端 parity (Metal+CUDA Design-2 迁移 + Fork-C 退休)",
          "timestamp": "2026-07-13T18:21:38+08:00",
          "tree_id": "a18a65f8c2734ab7abbdc729b38636720a51cf7b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/112183c86595221b64fc96eba2f3b6a5ba90d3b4"
        },
        "date": 1783938524718,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 423212.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 596095.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 401319.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 356370.4,
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
          "id": "86c3a72400dd77867bcdfc25f39d58157cb37d9e",
          "message": "Merge pull request #191 from LoveDaisy/fix/gpu-color-lane-multibatch-loss\n\nfix(gpu-color): device Y-lane accumulator persist across batches (multi-batch density loss)",
          "timestamp": "2026-07-14T08:28:32+08:00",
          "tree_id": "5210bc3b8aeb4997b8fe168b59b8e149bfe01690",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86c3a72400dd77867bcdfc25f39d58157cb37d9e"
        },
        "date": 1783989356775,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 476924.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588225.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 386953.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 319367.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "72f7619e19cd36fe102149a76d53fe42c350d600",
          "message": "Merge pull request #192 from LoveDaisy/feat/raypath-color-gui-polish-4\n\nfeat(raypath-color-gui): polish-4 UX 打磨 + 机械债扫尾 (scrum-360)",
          "timestamp": "2026-07-14T13:25:31+08:00",
          "tree_id": "cdf71a2e216f076b14a1ab4c78f740627e0934c8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/72f7619e19cd36fe102149a76d53fe42c350d600"
        },
        "date": 1784007181410,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 374738.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 590409.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 408073.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 337569.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8aed5ad07364407897caf3720afcc3ce9154de1c",
          "message": "Merge pull request #193 from LoveDaisy/refactor/filter-grammar-unify\n\nrefactor(gui): 统一 filter 语法 validate/parse 的 flush_ee 遍历骨架",
          "timestamp": "2026-07-14T17:04:03+08:00",
          "tree_id": "748625f849c992f0faab416aab89ff37884ed06d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8aed5ad07364407897caf3720afcc3ce9154de1c"
        },
        "date": 1784020304493,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 451854.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588136.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 407676.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 346796.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5913d079de3656d3730020f9968d7047cfcdeb3f",
          "message": "Merge pull request #194 from LoveDaisy/fix/gui-test-lifecycle-coroutine-gl\n\nfix(gui-test): guard optimistic_async_stop against no-GL-context coroutine upload",
          "timestamp": "2026-07-14T18:56:18+08:00",
          "tree_id": "c3f530adc1861772f9d4dbccfde72308aee3dc4c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5913d079de3656d3730020f9968d7047cfcdeb3f"
        },
        "date": 1784027015087,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 395733.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 593153.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 387320.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 349202.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "20112ea074c4336866f0877a7e8247f1f897c1f1",
          "message": "Merge pull request #195 from LoveDaisy/fix/gui-view-lens-no-resim\n\nfix(gui-state): view/lens/hemisphere changes must not trigger re-sim",
          "timestamp": "2026-07-15T00:23:26+08:00",
          "tree_id": "2a7de6a9dc50ccfc7d036982abeb09fc8c8215bb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/20112ea074c4336866f0877a7e8247f1f897c1f1"
        },
        "date": 1784046633207,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 308500.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592981.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 471731.3,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 319194.9,
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
          "id": "3c2653c6524e9c7b644754cf4c13d3b6d1554999",
          "message": "Merge pull request #196 from LoveDaisy/fix/gpu-parity-residual-debt\n\nfix(gpu-parity-residual-debt): 清 raypath-color GPU parity 残余债 (scrum-362)",
          "timestamp": "2026-07-15T08:02:45+08:00",
          "tree_id": "a88d7bea3761d9b539cc3074af4b1290e2cec8ef",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3c2653c6524e9c7b644754cf4c13d3b6d1554999"
        },
        "date": 1784074232978,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 373298.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592439.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 384776.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 272833.6,
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7fb0d1f4478fc0f63a7a5e2c5fbde2d6a7e073d2",
          "message": "Merge pull request #145 from LoveDaisy/chore/cleanup-metal-compile-review-minors\n\nchore: clean up Metal compile-path review minors (task-282/283 衍生, #293)",
          "timestamp": "2026-06-24T10:28:16+08:00",
          "tree_id": "c31eea8c0d7df06b3064a38d766c0960feaa9fde",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7fb0d1f4478fc0f63a7a5e2c5fbde2d6a7e073d2"
        },
        "date": 1782268467012,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 908158.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1256456.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 672625.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 630353.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2f8eb7bbab1521ea2da2b78755c8afe7e394a804",
          "message": "Merge pull request #146 from LoveDaisy/chore/release-prep\n\nchore: release prep v4.3.1 (perf-doc baseline + TraceCrystalBatch cleanup)",
          "timestamp": "2026-06-24T11:39:57+08:00",
          "tree_id": "3dca5e48d31d31d09f763da6c0d5dcc69e2ac0e7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2f8eb7bbab1521ea2da2b78755c8afe7e394a804"
        },
        "date": 1782272733334,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 768230.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1238073.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 685249,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 632260.2,
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
          "id": "19cbaf77d4bb88b85b6d5ed6f6330d0d19d86966",
          "message": "Merge pull request #147 from LoveDaisy/feat/cuda-backend-mvp\n\nfeat(gpu): CUDA backend MVP — single-MS no-filter raw-XYZ parity (scrum-#295)",
          "timestamp": "2026-06-25T15:24:53+08:00",
          "tree_id": "76d63a8f08faefd488e6b75f7425f402972e0d4d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/19cbaf77d4bb88b85b6d5ed6f6330d0d19d86966"
        },
        "date": 1782372672222,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 831313.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1252627.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 704187.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 571902,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5c9c62a9719c8db0cd540c81261e3b3d55b6f615",
          "message": "Merge pull request #148 from LoveDaisy/feat/cuda-backend-complete\n\nfeat(cuda): CUDA backend complete (scrum-296) — Metal 功能对齐 + 吞吐就绪",
          "timestamp": "2026-06-26T20:53:54+08:00",
          "tree_id": "b5aacabd73d8a1a44c91f040ab1ef998e00536d0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c9c62a9719c8db0cd540c81261e3b3d55b6f615"
        },
        "date": 1782478806841,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 927501.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1192917.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 688174.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 620163.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5710d6cd13a3290e0aaad7d92b4ac9f2a549a332",
          "message": "Merge pull request #149 from LoveDaisy/worktree-fix-stats-ray-count-u32-overflow\n\nfix(stats): widen ray-count types to 64-bit (Windows u32 overflow) — task-297",
          "timestamp": "2026-06-26T21:15:50+08:00",
          "tree_id": "1fcf076eb29b52ca6f7e11c988fee4e7340dbaa5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5710d6cd13a3290e0aaad7d92b4ac9f2a549a332"
        },
        "date": 1782480107931,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 914438.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1214645.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 671310,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 631878.5,
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
          "id": "df47f8ce94980421ea457cf2a81983343c84732b",
          "message": "Merge pull request #150 from LoveDaisy/chore/deferred-quality-cleanup\n\nchore: deferred quality cleanup (scrum-298) — e2e ref regen + geometry predicate single-source + ray_num float precision",
          "timestamp": "2026-06-26T22:50:03+08:00",
          "tree_id": "638406a85c800d2be16737ae7522efb8951a83fd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/df47f8ce94980421ea457cf2a81983343c84732b"
        },
        "date": 1782485778482,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 643663.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1208429.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 668678.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 609847,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7bd4807017bdde3f3cb961988b4ab9991eeec42b",
          "message": "Merge pull request #152 from LoveDaisy/feat/cuda-multi-ci-correctness\n\nfix(gpu): CUDA full multi-CI correctness + device-side recombine shuffle (Metal+CUDA)",
          "timestamp": "2026-06-27T21:44:16+08:00",
          "tree_id": "afa0020517cdb071aab346c7fde6ef574372c0f9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7bd4807017bdde3f3cb961988b4ab9991eeec42b"
        },
        "date": 1782568235104,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 635369.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1197058.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 668617.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 576019.8,
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
          "id": "bc66f2ec2fd37503c13844019d347f39549e4228",
          "message": "Merge pull request #153 from LoveDaisy/feat/gpu-device-fused-accumulation\n\nfeat(scrum-302): device-fused XYZ accumulation (Metal + CUDA)",
          "timestamp": "2026-06-28T10:45:30+08:00",
          "tree_id": "44b7f418587aa18fc23e29305d3eeda8bd8bb3c9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/bc66f2ec2fd37503c13844019d347f39549e4228"
        },
        "date": 1782615127410,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 918924.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1206589.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 671741.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 607007.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c2c5801cbe82410b1423d904c37480dc4ec03185",
          "message": "Merge pull request #154 from LoveDaisy/feat/cuda-async-engine-port\n\nperf(scrum-304): persist CUDA buffers across sessions — CUDA throughput competitive + bench standardized",
          "timestamp": "2026-06-29T09:28:47+08:00",
          "tree_id": "539bc1c7bc7672f023725ee7be21bafaa4fb88d5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c2c5801cbe82410b1423d904c37480dc4ec03185"
        },
        "date": 1782696881521,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 900255.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1197321.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 655185.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 606713.4,
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
          "id": "3c13a634c66560579a93d2b87cccd690510b1c83",
          "message": "Merge pull request #155 from LoveDaisy/feat/cuda-async-engine\n\nperf(scrum-306): CUDA throughput 37M→~114M (dispatch default + dead-buffer cap)",
          "timestamp": "2026-06-29T17:22:12+08:00",
          "tree_id": "0bfd23fcfaa5e3614e447a2936ea2d73bc2879af",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3c13a634c66560579a93d2b87cccd690510b1c83"
        },
        "date": 1782725333705,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 794557.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1202786,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 672845.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 578673.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "85c35073907dd5ea3eb9e9bf64bdcb20be8d0ac9",
          "message": "Merge pull request #156 from LoveDaisy/fix/randomsample-nomatch-entry-leak\n\nfix(geo3d): RandomSample no-match fallback for MSVC 77H light leak (curr_p==0.0 → entry-face bug)",
          "timestamp": "2026-06-30T14:48:02+08:00",
          "tree_id": "31d7f3bcd82d107a3ecd3e02f1be0e8105ec0277",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85c35073907dd5ea3eb9e9bf64bdcb20be8d0ac9"
        },
        "date": 1782802423185,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 680332.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1194690.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 670099.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 613953.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "270ec637801ed3a00639faf6e210e2a2a239c19a",
          "message": "Merge pull request #157 from LoveDaisy/feat/cuda-windows-validation\n\nCUDA on Windows: validation (#309) + delivery cluster (#310)",
          "timestamp": "2026-07-01T09:10:33+08:00",
          "tree_id": "2898159b77ce8341ec472358b0b3160cc9f7d1f2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/270ec637801ed3a00639faf6e210e2a2a239c19a"
        },
        "date": 1782868597982,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 637034.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1183032.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 664732.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 611289.5,
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
          "id": "9d189a99ead4ae89afa02c1323a666f5ef9105dc",
          "message": "build(deps): bump actions/cache from 5 to 6\n\nBumps [actions/cache](https://github.com/actions/cache) from 5 to 6.\n- [Release notes](https://github.com/actions/cache/releases)\n- [Changelog](https://github.com/actions/cache/blob/main/RELEASES.md)\n- [Commits](https://github.com/actions/cache/compare/v5...v6)\n\n---\nupdated-dependencies:\n- dependency-name: actions/cache\n  dependency-version: '6'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-07-01T10:01:02+08:00",
          "tree_id": "9dfc76647b46f224989c5ce3bbb595c48119842a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9d189a99ead4ae89afa02c1323a666f5ef9105dc"
        },
        "date": 1782871620205,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 593032.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1191223,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 842525.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 610633.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "085d3a1ef63b9ff2eba44ab57ad6b3d40cac33d1",
          "message": "Merge pull request #158 from LoveDaisy/feat/gpu-misc\n\nchore(cleanup): CUDA dead-code + CI Node24 bump + exit-seam crystals stat fix (scrum-311)",
          "timestamp": "2026-07-01T13:10:24+08:00",
          "tree_id": "6524533648ef89053374c3911be4dec2d1722643",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/085d3a1ef63b9ff2eba44ab57ad6b3d40cac33d1"
        },
        "date": 1782882989510,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 738717.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1194049.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 670186.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 546530.3,
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
          "id": "f8cdfeb59aa4b54dbb45e9bafbdfec1ea6176396",
          "message": "Merge pull request #159 from LoveDaisy/feat/gpu-readback-third-clock\n\nfeat(gpu): third-clock readback decoupling — fix high-resolution GPU throughput",
          "timestamp": "2026-07-01T21:22:40+08:00",
          "tree_id": "ce2f065d0985c3fc1f1e81c395b8260d8d1498b9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f8cdfeb59aa4b54dbb45e9bafbdfec1ea6176396"
        },
        "date": 1782912538192,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 686162.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1195125.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 713521.8,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 604394.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7010f091e1a9f26c946233253304c160df89a095",
          "message": "Merge pull request #160 from LoveDaisy/chore/gpu-doc-consolidation\n\ndocs+bench: GPU doc consolidation + collapse GPU --benchmark to one steady pass",
          "timestamp": "2026-07-02T09:15:42+08:00",
          "tree_id": "ce207ad8fbe6fc8a13cd0d39fd8d4dc784aba9c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7010f091e1a9f26c946233253304c160df89a095"
        },
        "date": 1782955306684,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 843159.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1200898.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 664001.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 618900.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "85fdfef28b6cbde3034622ca889e9d508457ca0d",
          "message": "Merge pull request #162 from LoveDaisy/feat/gpu-projection-parity\n\nfeat(gpu): unify render projection into single source + all 11 projections on Metal/CUDA (scrum-315)",
          "timestamp": "2026-07-02T14:58:05+08:00",
          "tree_id": "94499fa0850c528ab90de71578462a098aae6efb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85fdfef28b6cbde3034622ca889e9d508457ca0d"
        },
        "date": 1782975850251,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 706973,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1190231.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 648114.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 593539.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "23f16349a76ae1ea75539389ae8c1beb8c83b93e",
          "message": "Merge pull request #163 from LoveDaisy/ci/parallelize-slow-e2e\n\nci: parallelize slow-e2e with pytest-xdist, isolate throughput gates",
          "timestamp": "2026-07-02T17:42:26+08:00",
          "tree_id": "6097dda8ad93d8a7322ae8086936166b9c7ecbd1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/23f16349a76ae1ea75539389ae8c1beb8c83b93e"
        },
        "date": 1782985719377,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 880401.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1197900.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 1031501.6,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) 6973P-C\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 576447.1,
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
          "id": "ba2b6bbf58cb0df294c2994d9af0c0c39a8fe3d4",
          "message": "Merge pull request #161 from LoveDaisy/feat/gpu-bench-drain-aligned-rate\n\nfix(bench): drain-count-driven GPU --benchmark rate (fixes 5× under-report)",
          "timestamp": "2026-07-02T17:54:37+08:00",
          "tree_id": "5f5489b014b23a7dbed42902f7ac0d35530eb9a9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ba2b6bbf58cb0df294c2994d9af0c0c39a8fe3d4"
        },
        "date": 1782986445560,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 935715.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1204438.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 780632.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 675003.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "941ef7f1f57c56309fd9833e8bc4ecedcdb7914c",
          "message": "Merge pull request #164 from LoveDaisy/feat/gpu-rng-ray-index-uint64\n\nfix(gpu-rng): lift device-gen PCG ray-index 32-bit cap (uint64 lo/hi)",
          "timestamp": "2026-07-03T00:46:46+08:00",
          "tree_id": "b4bda3ea0cc4d5a7add7d4f7445183855fe24710",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/941ef7f1f57c56309fd9833e8bc4ecedcdb7914c"
        },
        "date": 1783011185891,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 848895.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1219478.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 774951.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 646650.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c9d7885ae89b30fe1a6ebf168686aa712613d18d",
          "message": "Merge pull request #165 from LoveDaisy/feat/gui-cli-render-alignment\n\nfix(gui): align GUI preview lens orientation with CLI render (scrum-320)",
          "timestamp": "2026-07-03T10:43:42+08:00",
          "tree_id": "690cea094f72fb75ca121307df859bfdf6ea2575",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c9d7885ae89b30fe1a6ebf168686aa712613d18d"
        },
        "date": 1783047010818,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1134134.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1208994.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 787111.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 675085.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f5b75ab9e83d57e35e755c4898cff59adaaf1faf",
          "message": "Merge pull request #166 from LoveDaisy/feat/azimuth-handedness-alignment\n\nfix(render): unify screen handedness to right=+az (scrum-321)",
          "timestamp": "2026-07-03T16:07:56+08:00",
          "tree_id": "83dad4392b494a6bad816129042c2a0250c83832",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5b75ab9e83d57e35e755c4898cff59adaaf1faf"
        },
        "date": 1783066440790,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 769818.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1221481.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 789751.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 671187.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fd719cc6d151b919a44e8ff1717f6e393a9bb12c",
          "message": "Merge pull request #167 from LoveDaisy/feat/gui-lifecycle-clock-decouple\n\nGUI preview lifecycle: clock-decouple to single-source epoch/lifecycle (I1–I6)",
          "timestamp": "2026-07-03T16:35:42+08:00",
          "tree_id": "6e1aaabdbcb0ad556df9d66266d4156f23f8c2dd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fd719cc6d151b919a44e8ff1717f6e393a9bb12c"
        },
        "date": 1783068104411,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1000910.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1210261.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 743737.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 719100.3,
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
          "id": "938964638e672bf93d079a6380a9a2e136258f18",
          "message": "Merge pull request #168 from LoveDaisy/feat/task-gui-custom-spectrum\n\nfeat(gui): custom discrete spectrum editor + ray_num total-across-wavelengths semantics (task-323)",
          "timestamp": "2026-07-04T00:50:09+08:00",
          "tree_id": "d051e01b6ab81a4e3032c05d1af88fb45e1c9b93",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/938964638e672bf93d079a6380a9a2e136258f18"
        },
        "date": 1783097777417,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 869283.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1222524.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 786156.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 631565.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "026928a679c7e511723436c1dd4d8f3ae18c16ab",
          "message": "Merge pull request #169 from LoveDaisy/feat/gui-ms-prob-footguns\n\ngui: MS layer prob footgun guards (four-state slider, +Layer promotion, CLI warning)",
          "timestamp": "2026-07-04T01:22:18+08:00",
          "tree_id": "8ebb7d1b8443f8baf800439ff34a54dfd2e8a09e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/026928a679c7e511723436c1dd4d8f3ae18c16ab"
        },
        "date": 1783099705078,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 957710.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1226542.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 775015.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 670852.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "32962163bd51949dbeed818117d5126fdc0b35f3",
          "message": "Merge pull request #171 from LoveDaisy/feat/near-pole-area-measure-sampling\n\nfix(gpu): root-fix near-pole rejection waste via unified tight-envelope area-measure sampling (scrum-328)",
          "timestamp": "2026-07-04T19:38:44+08:00",
          "tree_id": "471a6c43b8b361b747c4f824aa4b6910c5746a76",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/32962163bd51949dbeed818117d5126fdc0b35f3"
        },
        "date": 1783165507591,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1251464.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1215201.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 783127.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 667610.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "21cba8ceb52eb890be158eb7ae8abf66a3036414",
          "message": "Merge pull request #172 from LoveDaisy/feat/capi-filter-typed-commit\n\nC API filter typed-struct commit convergence (327) + backend-swap preview fix",
          "timestamp": "2026-07-04T21:12:29+08:00",
          "tree_id": "b0c7d73eda939b5e2e3ab1873a5585a6d28b8373",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/21cba8ceb52eb890be158eb7ae8abf66a3036414"
        },
        "date": 1783171120098,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1142601.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1222798.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 785725.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 675339.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "de22a871800c89cf92e502b71a29f91d134fb927",
          "message": "Merge pull request #173 from LoveDaisy/feat/gui-spectrum-modal-reset-button\n\nfeat(gui): Custom Spectrum modal Reset + overlimit warning polish (GUI small-fixes batch)",
          "timestamp": "2026-07-05T12:12:29+08:00",
          "tree_id": "b33e8efd36f103f5d98a7ee3ea26bf4e2008c972",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de22a871800c89cf92e502b71a29f91d134fb927"
        },
        "date": 1783225126067,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 935066,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1218810.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 743494.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 676201.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7c47f07a520ff7130d21cc6c16755c008e6e5c8d",
          "message": "Merge pull request #174 from LoveDaisy/feat/unify-orientation-sampling-cosine-measure\n\nUnify orientation latitude sampling to a cosine-measure inverse-CDF LUT",
          "timestamp": "2026-07-06T08:58:34+08:00",
          "tree_id": "5a7bb8bc6d9e114d130ac42451f7b3b85b9ea5ab",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7c47f07a520ff7130d21cc6c16755c008e6e5c8d"
        },
        "date": 1783299874748,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1225314.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1212673.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 797955,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 712433.1,
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
          "id": "aff094ef773a9d78983dc1d7d1aabf793757c22d",
          "message": "Merge pull request #175 from LoveDaisy/feat/raypath-color-foundation\n\nraypath-color foundation: per-ray component mask across CPU/Metal/CUDA (scrum-331)",
          "timestamp": "2026-07-06T09:26:54+08:00",
          "tree_id": "3cd3bee8f8ae3b0625d56d07253b29d0593b3f46",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/aff094ef773a9d78983dc1d7d1aabf793757c22d"
        },
        "date": 1783301594366,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 895624.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1176692,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 884335,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 640984.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b634945ce767a3531a89893427812f8f4a905607",
          "message": "Merge pull request #176 from LoveDaisy/chore/pre-release\n\nchore: pre-release housekeeping + raypath-color phase-3 blueprint",
          "timestamp": "2026-07-06T11:09:46+08:00",
          "tree_id": "9f67656a300d6ed2c850553edb269e19f5effc58",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b634945ce767a3531a89893427812f8f4a905607"
        },
        "date": 1783307747434,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 785235.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1187612.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 783213.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 596320.1,
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
          "id": "fc59c7253b6ae043676a2692ef0b8e55ea060405",
          "message": "Merge pull request #177 from LoveDaisy/feat/filter-editor-uplift\n\nfeat(gui): H5 sum-of-products filter editor + input ergonomics (scrum-333/334)",
          "timestamp": "2026-07-07T01:14:22+08:00",
          "tree_id": "adac9b7620bed310e892d5298d721d05c6f88500",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc59c7253b6ae043676a2692ef0b8e55ea060405"
        },
        "date": 1783358454696,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 979942.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1183819.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 784529.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 656262.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b7b53fef279f91c39cc06311da7ba1a5b5f52ceb",
          "message": "Merge pull request #178 from LoveDaisy/perf/latlut-shared-cache\n\nperf(latlut): fix mixed-axis multi-crystal LUT rebuild thrash (~20x)",
          "timestamp": "2026-07-07T09:10:44+08:00",
          "tree_id": "c5f07544b862c9153a6ffd644ad93eba311fa21a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b7b53fef279f91c39cc06311da7ba1a5b5f52ceb"
        },
        "date": 1783387061517,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 893430.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1178815.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 741247.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 615865.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3adb02485be2774b81a838d29d3e57f99be9cb2a",
          "message": "Merge pull request #179 from LoveDaisy/fix/crystal-preview-thumbnail\n\nfix(gui): correct crystal-preview face labels + reset pose on card switch (task-337)",
          "timestamp": "2026-07-07T11:36:30+08:00",
          "tree_id": "29de997cfb09809b633bfdc7d7d31c581fc1679d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3adb02485be2774b81a838d29d3e57f99be9cb2a"
        },
        "date": 1783395718549,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1186617.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1196171.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 775111.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 755768.9,
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
          "id": "e5c2ab33efb98c2c0d0100e30facccc50e2782ab",
          "message": "Merge pull request #180 from LoveDaisy/fix/modal-edit-state-leak\n\nfix(gui): stop edit-modal state leaking across crystal entries",
          "timestamp": "2026-07-07T15:41:11+08:00",
          "tree_id": "adc13365d8c0abdaf221ec7ba1ae23d7fae8414f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e5c2ab33efb98c2c0d0100e30facccc50e2782ab"
        },
        "date": 1783410483644,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 852478,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1184008.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 775621.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 640384.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8d88f08734051714d5bb893476504dc8be228f72",
          "message": "Merge pull request #181 from LoveDaisy/fix/regen-auto-ev-refs\n\nfix(auto-ev): regen stale visual refs + recalibrate thresholds (kill 31% flake)",
          "timestamp": "2026-07-07T22:15:20+08:00",
          "tree_id": "41512c22064985ece3e71d5e151eba5e56362085",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d88f08734051714d5bb893476504dc8be228f72"
        },
        "date": 1783434137981,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1122432,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1188372.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 760775.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 636655.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "11e8a6b1b736e3f9efad2e54fb068a48b09283d3",
          "message": "Merge pull request #182 from LoveDaisy/feat/color-components\n\nfeat(raypath-color): per-raypath color engine — color-class schema + rule-lane compositor (CLI/core, CPU)",
          "timestamp": "2026-07-08T00:44:52+08:00",
          "tree_id": "7eb8f0d87c2ce98aeec8d87910abae09d1f52567",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/11e8a6b1b736e3f9efad2e54fb068a48b09283d3"
        },
        "date": 1783443127081,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 754424.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175516,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 873227.3,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 628045.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "79002e6775a0a895acab77df2ac4a9586a5e2105",
          "message": "Merge pull request #183 from LoveDaisy/perf/gui-test-fixed-dt\n\nperf(gui-test): decouple frame budget from wall-clock (--fixed-dt, 16x faster correctness pool)",
          "timestamp": "2026-07-08T16:52:29+08:00",
          "tree_id": "e4db6edaea0749753c8f12e09e42ae50187e3c48",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/79002e6775a0a895acab77df2ac4a9586a5e2105"
        },
        "date": 1783501109615,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1151080.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1183000.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 768139.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 789972,
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
          "id": "6bd70df3d6b1c12a3ad92d53e65d7c71dc5b9c75",
          "message": "Merge pull request #184 from LoveDaisy/feat/raypath-color-design2\n\nfeat(raypath-color): phase-3b Design-2 redirect + GUI color window + preview v1 + dynamic-ABI fix",
          "timestamp": "2026-07-08T23:06:24+08:00",
          "tree_id": "dd27c9f90fa94a0633710927433158a0da5b81df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6bd70df3d6b1c12a3ad92d53e65d7c71dc5b9c75"
        },
        "date": 1783523619197,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1141309.9,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1192618.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 741382.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 669593.8,
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
          "id": "3810ef19d13d584242a467c18b2a4f3236b91c9b",
          "message": "Merge pull request #185 from LoveDaisy/feat/raypath-color-gui-polish\n\nper-raypath 染色 GUI phase-3b polish（scrum-345/346 + task-347）",
          "timestamp": "2026-07-10T08:56:20+08:00",
          "tree_id": "aed23ac51a7a8611ce569fa549c3ffb93fa2d180",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3810ef19d13d584242a467c18b2a4f3236b91c9b"
        },
        "date": 1783645442082,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1130717,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1176917.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 778950.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 607579.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d44e3da99e84bc27666910b9d8bb3c2adb090ef6",
          "message": "Merge pull request #186 from LoveDaisy/feat/raypath-color-gui-polish-2\n\nper-raypath 染色 GUI polish（三轮 on-screen 反馈：状态提示/ergonomics/Open 旧图残留根治）",
          "timestamp": "2026-07-11T07:05:30+08:00",
          "tree_id": "69725b832ee1697cbe2d6864cbe7ecee780b549d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d44e3da99e84bc27666910b9d8bb3c2adb090ef6"
        },
        "date": 1783725210718,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 752524.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1172128.8,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 736510,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 586644.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5e45e77b49c06d5ed9f7ccc5f96770da8e6a371d",
          "message": "Merge pull request #187 from LoveDaisy/feat/gui-state-reconcile\n\nGUI 状态治理专项：统一状态转换范式（explore-352 → scrum-353 + 354/355）",
          "timestamp": "2026-07-12T13:55:17+08:00",
          "tree_id": "a31a72d3fb0389df9c193f47ffdf4a14a1e33b30",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5e45e77b49c06d5ed9f7ccc5f96770da8e6a371d"
        },
        "date": 1783836075111,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1090051.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1191614.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 782609.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 777285.2,
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
          "id": "3ddafb383f6c183c3af704f4b34391509bb2fe09",
          "message": "Merge pull request #188 from LoveDaisy/feat/color-predicate-symmetry\n\nfeat: colour predicate PBD symmetry (scrum-356)",
          "timestamp": "2026-07-12T19:48:34+08:00",
          "tree_id": "839ef5a7d5f888a3d0ea9d623af51b57ecc2caca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3ddafb383f6c183c3af704f4b34391509bb2fe09"
        },
        "date": 1783857381928,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1070633.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1185401.9,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 776458.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 659593.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5e97f32dfff1df95c079d3e6a6dcf1d31ff2b870",
          "message": "Merge pull request #189 from LoveDaisy/feat/local-cleanup-sweep\n\nchore: local cleanup sweep — popcount gate, sibling-race, filter test, doc fixup (scrum-357)",
          "timestamp": "2026-07-13T00:20:25+08:00",
          "tree_id": "c297e4cca53bbc42294ff1e88146c4bcf2c33696",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5e97f32dfff1df95c079d3e6a6dcf1d31ff2b870"
        },
        "date": 1783873679854,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1042452,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1169967.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 769719.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 631277.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "112183c86595221b64fc96eba2f3b6a5ba90d3b4",
          "message": "Merge pull request #190 from LoveDaisy/feat/raypath-color-gpu-parity\n\nphase-3c: GPU 染色三后端 parity (Metal+CUDA Design-2 迁移 + Fork-C 退休)",
          "timestamp": "2026-07-13T18:21:38+08:00",
          "tree_id": "a18a65f8c2734ab7abbdc729b38636720a51cf7b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/112183c86595221b64fc96eba2f3b6a5ba90d3b4"
        },
        "date": 1783938528508,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1087185.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1183069.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 787199.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 643282,
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
          "id": "86c3a72400dd77867bcdfc25f39d58157cb37d9e",
          "message": "Merge pull request #191 from LoveDaisy/fix/gpu-color-lane-multibatch-loss\n\nfix(gpu-color): device Y-lane accumulator persist across batches (multi-batch density loss)",
          "timestamp": "2026-07-14T08:28:32+08:00",
          "tree_id": "5210bc3b8aeb4997b8fe168b59b8e149bfe01690",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86c3a72400dd77867bcdfc25f39d58157cb37d9e"
        },
        "date": 1783989360905,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1179942.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1176576,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 738216.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 582263.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "72f7619e19cd36fe102149a76d53fe42c350d600",
          "message": "Merge pull request #192 from LoveDaisy/feat/raypath-color-gui-polish-4\n\nfeat(raypath-color-gui): polish-4 UX 打磨 + 机械债扫尾 (scrum-360)",
          "timestamp": "2026-07-14T13:25:31+08:00",
          "tree_id": "cdf71a2e216f076b14a1ab4c78f740627e0934c8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/72f7619e19cd36fe102149a76d53fe42c350d600"
        },
        "date": 1784007184343,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 957142.3,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1174039.2,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 780061.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 625861.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8aed5ad07364407897caf3720afcc3ce9154de1c",
          "message": "Merge pull request #193 from LoveDaisy/refactor/filter-grammar-unify\n\nrefactor(gui): 统一 filter 语法 validate/parse 的 flush_ee 遍历骨架",
          "timestamp": "2026-07-14T17:04:03+08:00",
          "tree_id": "748625f849c992f0faab416aab89ff37884ed06d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8aed5ad07364407897caf3720afcc3ce9154de1c"
        },
        "date": 1784020309014,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1149121.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1173915.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 779535.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 645804.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5913d079de3656d3730020f9968d7047cfcdeb3f",
          "message": "Merge pull request #194 from LoveDaisy/fix/gui-test-lifecycle-coroutine-gl\n\nfix(gui-test): guard optimistic_async_stop against no-GL-context coroutine upload",
          "timestamp": "2026-07-14T18:56:18+08:00",
          "tree_id": "c3f530adc1861772f9d4dbccfde72308aee3dc4c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5913d079de3656d3730020f9968d7047cfcdeb3f"
        },
        "date": 1784027018169,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 823376.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1175039.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 742694.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 661103.5,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "20112ea074c4336866f0877a7e8247f1f897c1f1",
          "message": "Merge pull request #195 from LoveDaisy/fix/gui-view-lens-no-resim\n\nfix(gui-state): view/lens/hemisphere changes must not trigger re-sim",
          "timestamp": "2026-07-15T00:23:26+08:00",
          "tree_id": "2a7de6a9dc50ccfc7d036982abeb09fc8c8215bb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/20112ea074c4336866f0877a7e8247f1f897c1f1"
        },
        "date": 1784046637951,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 810392.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1186916.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 870036.5,
            "unit": "rays/sec",
            "extra": "CPU: Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 607531.4,
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
          "id": "3c2653c6524e9c7b644754cf4c13d3b6d1554999",
          "message": "Merge pull request #196 from LoveDaisy/fix/gpu-parity-residual-debt\n\nfix(gpu-parity-residual-debt): 清 raypath-color GPU parity 残余债 (scrum-362)",
          "timestamp": "2026-07-15T08:02:45+08:00",
          "tree_id": "a88d7bea3761d9b539cc3074af4b1290e2cec8ef",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3c2653c6524e9c7b644754cf4c13d3b6d1554999"
        },
        "date": 1784074236307,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 795456.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1185032.3,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 737818.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 448151.9,
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7fb0d1f4478fc0f63a7a5e2c5fbde2d6a7e073d2",
          "message": "Merge pull request #145 from LoveDaisy/chore/cleanup-metal-compile-review-minors\n\nchore: clean up Metal compile-path review minors (task-282/283 衍生, #293)",
          "timestamp": "2026-06-24T10:28:16+08:00",
          "tree_id": "c31eea8c0d7df06b3064a38d766c0960feaa9fde",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7fb0d1f4478fc0f63a7a5e2c5fbde2d6a7e073d2"
        },
        "date": 1782268468568,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 85.4,
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
          "id": "2f8eb7bbab1521ea2da2b78755c8afe7e394a804",
          "message": "Merge pull request #146 from LoveDaisy/chore/release-prep\n\nchore: release prep v4.3.1 (perf-doc baseline + TraceCrystalBatch cleanup)",
          "timestamp": "2026-06-24T11:39:57+08:00",
          "tree_id": "3dca5e48d31d31d09f763da6c0d5dcc69e2ac0e7",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2f8eb7bbab1521ea2da2b78755c8afe7e394a804"
        },
        "date": 1782272734808,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 73.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 85.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "19cbaf77d4bb88b85b6d5ed6f6330d0d19d86966",
          "message": "Merge pull request #147 from LoveDaisy/feat/cuda-backend-mvp\n\nfeat(gpu): CUDA backend MVP — single-MS no-filter raw-XYZ parity (scrum-#295)",
          "timestamp": "2026-06-25T15:24:53+08:00",
          "tree_id": "76d63a8f08faefd488e6b75f7425f402972e0d4d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/19cbaf77d4bb88b85b6d5ed6f6330d0d19d86966"
        },
        "date": 1782372674081,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 70.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 93.3,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 87.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5c9c62a9719c8db0cd540c81261e3b3d55b6f615",
          "message": "Merge pull request #148 from LoveDaisy/feat/cuda-backend-complete\n\nfeat(cuda): CUDA backend complete (scrum-296) — Metal 功能对齐 + 吞吐就绪",
          "timestamp": "2026-06-26T20:53:54+08:00",
          "tree_id": "b5aacabd73d8a1a44c91f040ab1ef998e00536d0",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5c9c62a9719c8db0cd540c81261e3b3d55b6f615"
        },
        "date": 1782478809352,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 69.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 86.9,
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
          "id": "5710d6cd13a3290e0aaad7d92b4ac9f2a549a332",
          "message": "Merge pull request #149 from LoveDaisy/worktree-fix-stats-ray-count-u32-overflow\n\nfix(stats): widen ray-count types to 64-bit (Windows u32 overflow) — task-297",
          "timestamp": "2026-06-26T21:15:50+08:00",
          "tree_id": "1fcf076eb29b52ca6f7e11c988fee4e7340dbaa5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5710d6cd13a3290e0aaad7d92b4ac9f2a549a332"
        },
        "date": 1782480109553,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 67.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 86.6,
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
          "id": "df47f8ce94980421ea457cf2a81983343c84732b",
          "message": "Merge pull request #150 from LoveDaisy/chore/deferred-quality-cleanup\n\nchore: deferred quality cleanup (scrum-298) — e2e ref regen + geometry predicate single-source + ray_num float precision",
          "timestamp": "2026-06-26T22:50:03+08:00",
          "tree_id": "638406a85c800d2be16737ae7522efb8951a83fd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/df47f8ce94980421ea457cf2a81983343c84732b"
        },
        "date": 1782485780750,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 60.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 103.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 83.8,
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
          "id": "7bd4807017bdde3f3cb961988b4ab9991eeec42b",
          "message": "Merge pull request #152 from LoveDaisy/feat/cuda-multi-ci-correctness\n\nfix(gpu): CUDA full multi-CI correctness + device-side recombine shuffle (Metal+CUDA)",
          "timestamp": "2026-06-27T21:44:16+08:00",
          "tree_id": "afa0020517cdb071aab346c7fde6ef574372c0f9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7bd4807017bdde3f3cb961988b4ab9991eeec42b"
        },
        "date": 1782568237040,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 64.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 85.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 101.7,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bc66f2ec2fd37503c13844019d347f39549e4228",
          "message": "Merge pull request #153 from LoveDaisy/feat/gpu-device-fused-accumulation\n\nfeat(scrum-302): device-fused XYZ accumulation (Metal + CUDA)",
          "timestamp": "2026-06-28T10:45:30+08:00",
          "tree_id": "44b7f418587aa18fc23e29305d3eeda8bd8bb3c9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/bc66f2ec2fd37503c13844019d347f39549e4228"
        },
        "date": 1782615129259,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 70.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 85,
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
          "id": "c2c5801cbe82410b1423d904c37480dc4ec03185",
          "message": "Merge pull request #154 from LoveDaisy/feat/cuda-async-engine-port\n\nperf(scrum-304): persist CUDA buffers across sessions — CUDA throughput competitive + bench standardized",
          "timestamp": "2026-06-29T09:28:47+08:00",
          "tree_id": "539bc1c7bc7672f023725ee7be21bafaa4fb88d5",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c2c5801cbe82410b1423d904c37480dc4ec03185"
        },
        "date": 1782696882978,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 88.5,
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
          "id": "3c13a634c66560579a93d2b87cccd690510b1c83",
          "message": "Merge pull request #155 from LoveDaisy/feat/cuda-async-engine\n\nperf(scrum-306): CUDA throughput 37M→~114M (dispatch default + dead-buffer cap)",
          "timestamp": "2026-06-29T17:22:12+08:00",
          "tree_id": "0bfd23fcfaa5e3614e447a2936ea2d73bc2879af",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3c13a634c66560579a93d2b87cccd690510b1c83"
        },
        "date": 1782725335223,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 69.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 87.8,
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
          "id": "85c35073907dd5ea3eb9e9bf64bdcb20be8d0ac9",
          "message": "Merge pull request #156 from LoveDaisy/fix/randomsample-nomatch-entry-leak\n\nfix(geo3d): RandomSample no-match fallback for MSVC 77H light leak (curr_p==0.0 → entry-face bug)",
          "timestamp": "2026-06-30T14:48:02+08:00",
          "tree_id": "31d7f3bcd82d107a3ecd3e02f1be0e8105ec0277",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85c35073907dd5ea3eb9e9bf64bdcb20be8d0ac9"
        },
        "date": 1782802425087,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 86,
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
          "id": "270ec637801ed3a00639faf6e210e2a2a239c19a",
          "message": "Merge pull request #157 from LoveDaisy/feat/cuda-windows-validation\n\nCUDA on Windows: validation (#309) + delivery cluster (#310)",
          "timestamp": "2026-07-01T09:10:33+08:00",
          "tree_id": "2898159b77ce8341ec472358b0b3160cc9f7d1f2",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/270ec637801ed3a00639faf6e210e2a2a239c19a"
        },
        "date": 1782868599709,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 87.1,
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
          "id": "9d189a99ead4ae89afa02c1323a666f5ef9105dc",
          "message": "build(deps): bump actions/cache from 5 to 6\n\nBumps [actions/cache](https://github.com/actions/cache) from 5 to 6.\n- [Release notes](https://github.com/actions/cache/releases)\n- [Changelog](https://github.com/actions/cache/blob/main/RELEASES.md)\n- [Commits](https://github.com/actions/cache/compare/v5...v6)\n\n---\nupdated-dependencies:\n- dependency-name: actions/cache\n  dependency-version: '6'\n  dependency-type: direct:production\n  update-type: version-update:semver-major\n...\n\nSigned-off-by: dependabot[bot] <support@github.com>",
          "timestamp": "2026-07-01T10:01:02+08:00",
          "tree_id": "9dfc76647b46f224989c5ce3bbb595c48119842a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9d189a99ead4ae89afa02c1323a666f5ef9105dc"
        },
        "date": 1782871621798,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 68.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 103.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 88.8,
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
          "id": "085d3a1ef63b9ff2eba44ab57ad6b3d40cac33d1",
          "message": "Merge pull request #158 from LoveDaisy/feat/gpu-misc\n\nchore(cleanup): CUDA dead-code + CI Node24 bump + exit-seam crystals stat fix (scrum-311)",
          "timestamp": "2026-07-01T13:10:24+08:00",
          "tree_id": "6524533648ef89053374c3911be4dec2d1722643",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/085d3a1ef63b9ff2eba44ab57ad6b3d40cac33d1"
        },
        "date": 1782882991028,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 76.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 84.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 109.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f8cdfeb59aa4b54dbb45e9bafbdfec1ea6176396",
          "message": "Merge pull request #159 from LoveDaisy/feat/gpu-readback-third-clock\n\nfeat(gpu): third-clock readback decoupling — fix high-resolution GPU throughput",
          "timestamp": "2026-07-01T21:22:40+08:00",
          "tree_id": "ce2f065d0985c3fc1f1e81c395b8260d8d1498b9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f8cdfeb59aa4b54dbb45e9bafbdfec1ea6176396"
        },
        "date": 1782912539859,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 78.9,
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
          "id": "7010f091e1a9f26c946233253304c160df89a095",
          "message": "Merge pull request #160 from LoveDaisy/chore/gpu-doc-consolidation\n\ndocs+bench: GPU doc consolidation + collapse GPU --benchmark to one steady pass",
          "timestamp": "2026-07-02T09:15:42+08:00",
          "tree_id": "ce207ad8fbe6fc8a13cd0d39fd8d4dc784aba9c3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7010f091e1a9f26c946233253304c160df89a095"
        },
        "date": 1782955308860,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 64.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 84.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
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
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "85fdfef28b6cbde3034622ca889e9d508457ca0d",
          "message": "Merge pull request #162 from LoveDaisy/feat/gpu-projection-parity\n\nfeat(gpu): unify render projection into single source + all 11 projections on Metal/CUDA (scrum-315)",
          "timestamp": "2026-07-02T14:58:05+08:00",
          "tree_id": "94499fa0850c528ab90de71578462a098aae6efb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/85fdfef28b6cbde3034622ca889e9d508457ca0d"
        },
        "date": 1782975851890,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 73.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 88.4,
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
          "id": "23f16349a76ae1ea75539389ae8c1beb8c83b93e",
          "message": "Merge pull request #163 from LoveDaisy/ci/parallelize-slow-e2e\n\nci: parallelize slow-e2e with pytest-xdist, isolate throughput gates",
          "timestamp": "2026-07-02T17:42:26+08:00",
          "tree_id": "6097dda8ad93d8a7322ae8086936166b9c7ecbd1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/23f16349a76ae1ea75539389ae8c1beb8c83b93e"
        },
        "date": 1782985721554,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 62.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 102.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 87.3,
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
          "id": "ba2b6bbf58cb0df294c2994d9af0c0c39a8fe3d4",
          "message": "Merge pull request #161 from LoveDaisy/feat/gpu-bench-drain-aligned-rate\n\nfix(bench): drain-count-driven GPU --benchmark rate (fixes 5× under-report)",
          "timestamp": "2026-07-02T17:54:37+08:00",
          "tree_id": "5f5489b014b23a7dbed42902f7ac0d35530eb9a9",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/ba2b6bbf58cb0df294c2994d9af0c0c39a8fe3d4"
        },
        "date": 1782986447264,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 72.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.2,
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
          "id": "941ef7f1f57c56309fd9833e8bc4ecedcdb7914c",
          "message": "Merge pull request #164 from LoveDaisy/feat/gpu-rng-ray-index-uint64\n\nfix(gpu-rng): lift device-gen PCG ray-index 32-bit cap (uint64 lo/hi)",
          "timestamp": "2026-07-03T00:46:46+08:00",
          "tree_id": "b4bda3ea0cc4d5a7add7d4f7445183855fe24710",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/941ef7f1f57c56309fd9833e8bc4ecedcdb7914c"
        },
        "date": 1783011188172,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 89.6,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c9d7885ae89b30fe1a6ebf168686aa712613d18d",
          "message": "Merge pull request #165 from LoveDaisy/feat/gui-cli-render-alignment\n\nfix(gui): align GUI preview lens orientation with CLI render (scrum-320)",
          "timestamp": "2026-07-03T10:43:42+08:00",
          "tree_id": "690cea094f72fb75ca121307df859bfdf6ea2575",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/c9d7885ae89b30fe1a6ebf168686aa712613d18d"
        },
        "date": 1783047012546,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.2,
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
          "id": "f5b75ab9e83d57e35e755c4898cff59adaaf1faf",
          "message": "Merge pull request #166 from LoveDaisy/feat/azimuth-handedness-alignment\n\nfix(render): unify screen handedness to right=+az (scrum-321)",
          "timestamp": "2026-07-03T16:07:56+08:00",
          "tree_id": "83dad4392b494a6bad816129042c2a0250c83832",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/f5b75ab9e83d57e35e755c4898cff59adaaf1faf"
        },
        "date": 1783066442511,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 92.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fd719cc6d151b919a44e8ff1717f6e393a9bb12c",
          "message": "Merge pull request #167 from LoveDaisy/feat/gui-lifecycle-clock-decouple\n\nGUI preview lifecycle: clock-decouple to single-source epoch/lifecycle (I1–I6)",
          "timestamp": "2026-07-03T16:35:42+08:00",
          "tree_id": "6e1aaabdbcb0ad556df9d66266d4156f23f8c2dd",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fd719cc6d151b919a44e8ff1717f6e393a9bb12c"
        },
        "date": 1783068105935,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 88.3,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "938964638e672bf93d079a6380a9a2e136258f18",
          "message": "Merge pull request #168 from LoveDaisy/feat/task-gui-custom-spectrum\n\nfeat(gui): custom discrete spectrum editor + ray_num total-across-wavelengths semantics (task-323)",
          "timestamp": "2026-07-04T00:50:09+08:00",
          "tree_id": "d051e01b6ab81a4e3032c05d1af88fb45e1c9b93",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/938964638e672bf93d079a6380a9a2e136258f18"
        },
        "date": 1783097778914,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.3,
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
          "id": "026928a679c7e511723436c1dd4d8f3ae18c16ab",
          "message": "Merge pull request #169 from LoveDaisy/feat/gui-ms-prob-footguns\n\ngui: MS layer prob footgun guards (four-state slider, +Layer promotion, CLI warning)",
          "timestamp": "2026-07-04T01:22:18+08:00",
          "tree_id": "8ebb7d1b8443f8baf800439ff34a54dfd2e8a09e",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/026928a679c7e511723436c1dd4d8f3ae18c16ab"
        },
        "date": 1783099707239,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 79.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.2,
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
          "id": "32962163bd51949dbeed818117d5126fdc0b35f3",
          "message": "Merge pull request #171 from LoveDaisy/feat/near-pole-area-measure-sampling\n\nfix(gpu): root-fix near-pole rejection waste via unified tight-envelope area-measure sampling (scrum-328)",
          "timestamp": "2026-07-04T19:38:44+08:00",
          "tree_id": "471a6c43b8b361b747c4f824aa4b6910c5746a76",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/32962163bd51949dbeed818117d5126fdc0b35f3"
        },
        "date": 1783165509759,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.5,
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
          "id": "21cba8ceb52eb890be158eb7ae8abf66a3036414",
          "message": "Merge pull request #172 from LoveDaisy/feat/capi-filter-typed-commit\n\nC API filter typed-struct commit convergence (327) + backend-swap preview fix",
          "timestamp": "2026-07-04T21:12:29+08:00",
          "tree_id": "b0c7d73eda939b5e2e3ab1873a5585a6d28b8373",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/21cba8ceb52eb890be158eb7ae8abf66a3036414"
        },
        "date": 1783171121602,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.2,
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
          "id": "de22a871800c89cf92e502b71a29f91d134fb927",
          "message": "Merge pull request #173 from LoveDaisy/feat/gui-spectrum-modal-reset-button\n\nfeat(gui): Custom Spectrum modal Reset + overlimit warning polish (GUI small-fixes batch)",
          "timestamp": "2026-07-05T12:12:29+08:00",
          "tree_id": "b33e8efd36f103f5d98a7ee3ea26bf4e2008c972",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/de22a871800c89cf92e502b71a29f91d134fb927"
        },
        "date": 1783225127671,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 95.1,
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
          "id": "7c47f07a520ff7130d21cc6c16755c008e6e5c8d",
          "message": "Merge pull request #174 from LoveDaisy/feat/unify-orientation-sampling-cosine-measure\n\nUnify orientation latitude sampling to a cosine-measure inverse-CDF LUT",
          "timestamp": "2026-07-06T08:58:34+08:00",
          "tree_id": "5a7bb8bc6d9e114d130ac42451f7b3b85b9ea5ab",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/7c47f07a520ff7130d21cc6c16755c008e6e5c8d"
        },
        "date": 1783299876853,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.3,
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
          "id": "aff094ef773a9d78983dc1d7d1aabf793757c22d",
          "message": "Merge pull request #175 from LoveDaisy/feat/raypath-color-foundation\n\nraypath-color foundation: per-ray component mask across CPU/Metal/CUDA (scrum-331)",
          "timestamp": "2026-07-06T09:26:54+08:00",
          "tree_id": "3cd3bee8f8ae3b0625d56d07253b29d0593b3f46",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/aff094ef773a9d78983dc1d7d1aabf793757c22d"
        },
        "date": 1783301596505,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.4,
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
          "id": "b634945ce767a3531a89893427812f8f4a905607",
          "message": "Merge pull request #176 from LoveDaisy/chore/pre-release\n\nchore: pre-release housekeeping + raypath-color phase-3 blueprint",
          "timestamp": "2026-07-06T11:09:46+08:00",
          "tree_id": "9f67656a300d6ed2c850553edb269e19f5effc58",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b634945ce767a3531a89893427812f8f4a905607"
        },
        "date": 1783307749144,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
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
          "id": "fc59c7253b6ae043676a2692ef0b8e55ea060405",
          "message": "Merge pull request #177 from LoveDaisy/feat/filter-editor-uplift\n\nfeat(gui): H5 sum-of-products filter editor + input ergonomics (scrum-333/334)",
          "timestamp": "2026-07-07T01:14:22+08:00",
          "tree_id": "adac9b7620bed310e892d5298d721d05c6f88500",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/fc59c7253b6ae043676a2692ef0b8e55ea060405"
        },
        "date": 1783358456383,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 111.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.4,
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
          "id": "b7b53fef279f91c39cc06311da7ba1a5b5f52ceb",
          "message": "Merge pull request #178 from LoveDaisy/perf/latlut-shared-cache\n\nperf(latlut): fix mixed-axis multi-crystal LUT rebuild thrash (~20x)",
          "timestamp": "2026-07-07T09:10:44+08:00",
          "tree_id": "c5f07544b862c9153a6ffd644ad93eba311fa21a",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/b7b53fef279f91c39cc06311da7ba1a5b5f52ceb"
        },
        "date": 1783387063701,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.5,
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
          "id": "3adb02485be2774b81a838d29d3e57f99be9cb2a",
          "message": "Merge pull request #179 from LoveDaisy/fix/crystal-preview-thumbnail\n\nfix(gui): correct crystal-preview face labels + reset pose on card switch (task-337)",
          "timestamp": "2026-07-07T11:36:30+08:00",
          "tree_id": "29de997cfb09809b633bfdc7d7d31c581fc1679d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3adb02485be2774b81a838d29d3e57f99be9cb2a"
        },
        "date": 1783395720685,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 86.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.5,
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
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e5c2ab33efb98c2c0d0100e30facccc50e2782ab",
          "message": "Merge pull request #180 from LoveDaisy/fix/modal-edit-state-leak\n\nfix(gui): stop edit-modal state leaking across crystal entries",
          "timestamp": "2026-07-07T15:41:11+08:00",
          "tree_id": "adc13365d8c0abdaf221ec7ba1ae23d7fae8414f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e5c2ab33efb98c2c0d0100e30facccc50e2782ab"
        },
        "date": 1783410485310,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 89.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.4,
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
          "id": "8d88f08734051714d5bb893476504dc8be228f72",
          "message": "Merge pull request #181 from LoveDaisy/fix/regen-auto-ev-refs\n\nfix(auto-ev): regen stale visual refs + recalibrate thresholds (kill 31% flake)",
          "timestamp": "2026-07-07T22:15:20+08:00",
          "tree_id": "41512c22064985ece3e71d5e151eba5e56362085",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8d88f08734051714d5bb893476504dc8be228f72"
        },
        "date": 1783434139627,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.9,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96,
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
          "id": "11e8a6b1b736e3f9efad2e54fb068a48b09283d3",
          "message": "Merge pull request #182 from LoveDaisy/feat/color-components\n\nfeat(raypath-color): per-raypath color engine — color-class schema + rule-lane compositor (CLI/core, CPU)",
          "timestamp": "2026-07-08T00:44:52+08:00",
          "tree_id": "7eb8f0d87c2ce98aeec8d87910abae09d1f52567",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/11e8a6b1b736e3f9efad2e54fb068a48b09283d3"
        },
        "date": 1783443129118,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 91.9,
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
          "id": "79002e6775a0a895acab77df2ac4a9586a5e2105",
          "message": "Merge pull request #183 from LoveDaisy/perf/gui-test-fixed-dt\n\nperf(gui-test): decouple frame budget from wall-clock (--fixed-dt, 16x faster correctness pool)",
          "timestamp": "2026-07-08T16:52:29+08:00",
          "tree_id": "e4db6edaea0749753c8f12e09e42ae50187e3c48",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/79002e6775a0a895acab77df2ac4a9586a5e2105"
        },
        "date": 1783501112734,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 101.2,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6bd70df3d6b1c12a3ad92d53e65d7c71dc5b9c75",
          "message": "Merge pull request #184 from LoveDaisy/feat/raypath-color-design2\n\nfeat(raypath-color): phase-3b Design-2 redirect + GUI color window + preview v1 + dynamic-ABI fix",
          "timestamp": "2026-07-08T23:06:24+08:00",
          "tree_id": "dd27c9f90fa94a0633710927433158a0da5b81df",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6bd70df3d6b1c12a3ad92d53e65d7c71dc5b9c75"
        },
        "date": 1783523621539,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 89.7,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.8,
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
          "id": "3810ef19d13d584242a467c18b2a4f3236b91c9b",
          "message": "Merge pull request #185 from LoveDaisy/feat/raypath-color-gui-polish\n\nper-raypath 染色 GUI phase-3b polish（scrum-345/346 + task-347）",
          "timestamp": "2026-07-10T08:56:20+08:00",
          "tree_id": "aed23ac51a7a8611ce569fa549c3ffb93fa2d180",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3810ef19d13d584242a467c18b2a4f3236b91c9b"
        },
        "date": 1783645443954,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 83.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 91.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d44e3da99e84bc27666910b9d8bb3c2adb090ef6",
          "message": "Merge pull request #186 from LoveDaisy/feat/raypath-color-gui-polish-2\n\nper-raypath 染色 GUI polish（三轮 on-screen 反馈：状态提示/ergonomics/Open 旧图残留根治）",
          "timestamp": "2026-07-11T07:05:30+08:00",
          "tree_id": "69725b832ee1697cbe2d6864cbe7ecee780b549d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/d44e3da99e84bc27666910b9d8bb3c2adb090ef6"
        },
        "date": 1783725212259,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 65.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 98.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 86.9,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5e45e77b49c06d5ed9f7ccc5f96770da8e6a371d",
          "message": "Merge pull request #187 from LoveDaisy/feat/gui-state-reconcile\n\nGUI 状态治理专项：统一状态转换范式（explore-352 → scrum-353 + 354/355）",
          "timestamp": "2026-07-12T13:55:17+08:00",
          "tree_id": "a31a72d3fb0389df9c193f47ffdf4a14a1e33b30",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5e45e77b49c06d5ed9f7ccc5f96770da8e6a371d"
        },
        "date": 1783836076832,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 78.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.5,
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
          "id": "3ddafb383f6c183c3af704f4b34391509bb2fe09",
          "message": "Merge pull request #188 from LoveDaisy/feat/color-predicate-symmetry\n\nfeat: colour predicate PBD symmetry (scrum-356)",
          "timestamp": "2026-07-12T19:48:34+08:00",
          "tree_id": "839ef5a7d5f888a3d0ea9d623af51b57ecc2caca",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/3ddafb383f6c183c3af704f4b34391509bb2fe09"
        },
        "date": 1783857384304,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 81.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97.2,
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
          "id": "5e97f32dfff1df95c079d3e6a6dcf1d31ff2b870",
          "message": "Merge pull request #189 from LoveDaisy/feat/local-cleanup-sweep\n\nchore: local cleanup sweep — popcount gate, sibling-race, filter test, doc fixup (scrum-357)",
          "timestamp": "2026-07-13T00:20:25+08:00",
          "tree_id": "c297e4cca53bbc42294ff1e88146c4bcf2c33696",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5e97f32dfff1df95c079d3e6a6dcf1d31ff2b870"
        },
        "date": 1783873681451,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 77.3,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 98.6,
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
          "id": "112183c86595221b64fc96eba2f3b6a5ba90d3b4",
          "message": "Merge pull request #190 from LoveDaisy/feat/raypath-color-gpu-parity\n\nphase-3c: GPU 染色三后端 parity (Metal+CUDA Design-2 迁移 + Fork-C 退休)",
          "timestamp": "2026-07-13T18:21:38+08:00",
          "tree_id": "a18a65f8c2734ab7abbdc729b38636720a51cf7b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/112183c86595221b64fc96eba2f3b6a5ba90d3b4"
        },
        "date": 1783938530433,
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
            "value": 98.1,
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
          "id": "86c3a72400dd77867bcdfc25f39d58157cb37d9e",
          "message": "Merge pull request #191 from LoveDaisy/fix/gpu-color-lane-multibatch-loss\n\nfix(gpu-color): device Y-lane accumulator persist across batches (multi-batch density loss)",
          "timestamp": "2026-07-14T08:28:32+08:00",
          "tree_id": "5210bc3b8aeb4997b8fe168b59b8e149bfe01690",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/86c3a72400dd77867bcdfc25f39d58157cb37d9e"
        },
        "date": 1783989363495,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 82.5,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.4,
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
          "id": "72f7619e19cd36fe102149a76d53fe42c350d600",
          "message": "Merge pull request #192 from LoveDaisy/feat/raypath-color-gui-polish-4\n\nfeat(raypath-color-gui): polish-4 UX 打磨 + 机械债扫尾 (scrum-360)",
          "timestamp": "2026-07-14T13:25:31+08:00",
          "tree_id": "cdf71a2e216f076b14a1ab4c78f740627e0934c8",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/72f7619e19cd36fe102149a76d53fe42c350d600"
        },
        "date": 1784007185931,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 85.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.6,
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
          "id": "8aed5ad07364407897caf3720afcc3ce9154de1c",
          "message": "Merge pull request #193 from LoveDaisy/refactor/filter-grammar-unify\n\nrefactor(gui): 统一 filter 语法 validate/parse 的 flush_ee 遍历骨架",
          "timestamp": "2026-07-14T17:04:03+08:00",
          "tree_id": "748625f849c992f0faab416aab89ff37884ed06d",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/8aed5ad07364407897caf3720afcc3ce9154de1c"
        },
        "date": 1784020311268,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 84.8,
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
          "id": "5913d079de3656d3730020f9968d7047cfcdeb3f",
          "message": "Merge pull request #194 from LoveDaisy/fix/gui-test-lifecycle-coroutine-gl\n\nfix(gui-test): guard optimistic_async_stop against no-GL-context coroutine upload",
          "timestamp": "2026-07-14T18:56:18+08:00",
          "tree_id": "c3f530adc1861772f9d4dbccfde72308aee3dc4c",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/5913d079de3656d3730020f9968d7047cfcdeb3f"
        },
        "date": 1784027019926,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 69.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
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
          "id": "20112ea074c4336866f0877a7e8247f1f897c1f1",
          "message": "Merge pull request #195 from LoveDaisy/fix/gui-view-lens-no-resim\n\nfix(gui-state): view/lens/hemisphere changes must not trigger re-sim",
          "timestamp": "2026-07-15T00:23:26+08:00",
          "tree_id": "2a7de6a9dc50ccfc7d036982abeb09fc8c8215bb",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/20112ea074c4336866f0877a7e8247f1f897c1f1"
        },
        "date": 1784046640519,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 87.6,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.1,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 92.2,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.2,
            "unit": "%"
          }
        ]
      }
    ]
  }
}