window.BENCHMARK_DATA = {
  "lastUpdate": 1785983770373,
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6a2fa9ca79842740355bd10165ea2eb02cd8f279",
          "message": "Merge pull request #197 from LoveDaisy/feat/metal-gui-commit-backpressure\n\nfix(gui): Metal GUI commit backpressure — O2 PSO 进程级缓存 + 自适应背压门",
          "timestamp": "2026-07-15T10:32:02+08:00",
          "tree_id": "44dfcceb98f7237b697f2a21d2028562645450e3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6a2fa9ca79842740355bd10165ea2eb02cd8f279"
        },
        "date": 1784083189709,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 318808.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 590748.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 400145.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 329726.9,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e19100c3ddb56b6fd6824dc6ef55ec580d076639",
          "message": "Merge pull request #198 from LoveDaisy/fix/gpu-color-mask-batch-leak\n\nfix(gpu): layer-0 color-class mask cross-batch leak (Metal + CUDA)",
          "timestamp": "2026-07-15T13:17:01+08:00",
          "tree_id": "c9be5f895a1ea8318033db8cb4029ccd31fb54f1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e19100c3ddb56b6fd6824dc6ef55ec580d076639"
        },
        "date": 1784093076268,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 365654.8,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 596083.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 491972.3,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345998.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2cbcffd86aaaf5817d43b9b2504d063d705d802c",
          "message": "Merge pull request #199 from LoveDaisy/feat/painter-alpha-over\n\nfeat(painter-composite): painter 改亮度即 alpha 的 Porter-Duff over 合成 + 设默认",
          "timestamp": "2026-07-15T15:15:11+08:00",
          "tree_id": "8745cf535cc23d151ae929caa4af4ab41d872d10",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2cbcffd86aaaf5817d43b9b2504d063d705d802c"
        },
        "date": 1784100154279,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 363334.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 588916.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 495071.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 318488.5,
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
          "id": "e8f266452c9b204f5f067a0e9b7de29251e583e6",
          "message": "Merge pull request #200 from LoveDaisy/test/painter-default-e2e-coverage\n\ntest(painter-default-e2e): 补 painter 默认合成模式的 e2e 全链路覆盖",
          "timestamp": "2026-07-15T16:30:15+08:00",
          "tree_id": "e9f46c194ddee502cab098deb32fc54ec94eba1b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e8f266452c9b204f5f067a0e9b7de29251e583e6"
        },
        "date": 1784104653822,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 454060.2,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592068.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 406272.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 354522.1,
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
          "id": "9e05cea90d94df75e7d0bf0cd62dce574afaffaa",
          "message": "Merge pull request #201 from LoveDaisy/chore/classify-pixels-stale-docstring\n\ndocs(image_utils): 修正 classify_pixels_by_color_direction 过时 docstring",
          "timestamp": "2026-07-15T17:49:19+08:00",
          "tree_id": "b15db229e3139294d532f532c945a31d5c0b255f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9e05cea90d94df75e7d0bf0cd62dce574afaffaa"
        },
        "date": 1784109418234,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 327883.5,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 592517.5,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 382827.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 345125.7,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6a2fa9ca79842740355bd10165ea2eb02cd8f279",
          "message": "Merge pull request #197 from LoveDaisy/feat/metal-gui-commit-backpressure\n\nfix(gui): Metal GUI commit backpressure — O2 PSO 进程级缓存 + 自适应背压门",
          "timestamp": "2026-07-15T10:32:02+08:00",
          "tree_id": "44dfcceb98f7237b697f2a21d2028562645450e3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6a2fa9ca79842740355bd10165ea2eb02cd8f279"
        },
        "date": 1784083193949,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 772482.6,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1191106.1,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 776415.2,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 618098.8,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e19100c3ddb56b6fd6824dc6ef55ec580d076639",
          "message": "Merge pull request #198 from LoveDaisy/fix/gpu-color-mask-batch-leak\n\nfix(gpu): layer-0 color-class mask cross-batch leak (Metal + CUDA)",
          "timestamp": "2026-07-15T13:17:01+08:00",
          "tree_id": "c9be5f895a1ea8318033db8cb4029ccd31fb54f1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e19100c3ddb56b6fd6824dc6ef55ec580d076639"
        },
        "date": 1784093079555,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 823259.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1185104.6,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 949383,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 644501.4,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2cbcffd86aaaf5817d43b9b2504d063d705d802c",
          "message": "Merge pull request #199 from LoveDaisy/feat/painter-alpha-over\n\nfeat(painter-composite): painter 改亮度即 alpha 的 Porter-Duff over 合成 + 设默认",
          "timestamp": "2026-07-15T15:15:11+08:00",
          "tree_id": "8745cf535cc23d151ae929caa4af4ab41d872d10",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2cbcffd86aaaf5817d43b9b2504d063d705d802c"
        },
        "date": 1784100157102,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 723534.1,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1182461,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 947133.1,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 607847.2,
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
          "id": "e8f266452c9b204f5f067a0e9b7de29251e583e6",
          "message": "Merge pull request #200 from LoveDaisy/test/painter-default-e2e-coverage\n\ntest(painter-default-e2e): 补 painter 默认合成模式的 e2e 全链路覆盖",
          "timestamp": "2026-07-15T16:30:15+08:00",
          "tree_id": "e9f46c194ddee502cab098deb32fc54ec94eba1b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e8f266452c9b204f5f067a0e9b7de29251e583e6"
        },
        "date": 1784104658206,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 1094978.4,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1181525.4,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 778673,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 655555,
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
          "id": "9e05cea90d94df75e7d0bf0cd62dce574afaffaa",
          "message": "Merge pull request #201 from LoveDaisy/chore/classify-pixels-stale-docstring\n\ndocs(image_utils): 修正 classify_pixels_by_color_direction 过时 docstring",
          "timestamp": "2026-07-15T17:49:19+08:00",
          "tree_id": "b15db229e3139294d532f532c945a31d5c0b255f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9e05cea90d94df75e7d0bf0cd62dce574afaffaa"
        },
        "date": 1784109421316,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 745763.7,
            "unit": "rays/sec",
            "extra": "CPU: Apple M1 (Virtual)\\nCores: 3"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 1184545.7,
            "unit": "rays/sec",
            "extra": "CPU: Neoverse-N2\\nCores: 4"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 735165,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 9V74 80-Core Processor\\nCores: 4"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 653681.6,
            "unit": "rays/sec",
            "extra": "CPU: AMD EPYC 7763 64-Core Processor                \\nCores: 4"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
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
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
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
        "date": 1784074237887,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 71,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.9,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 82.1,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6a2fa9ca79842740355bd10165ea2eb02cd8f279",
          "message": "Merge pull request #197 from LoveDaisy/feat/metal-gui-commit-backpressure\n\nfix(gui): Metal GUI commit backpressure — O2 PSO 进程级缓存 + 自适应背压门",
          "timestamp": "2026-07-15T10:32:02+08:00",
          "tree_id": "44dfcceb98f7237b697f2a21d2028562645450e3",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/6a2fa9ca79842740355bd10165ea2eb02cd8f279"
        },
        "date": 1784083196117,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 80.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.8,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 97,
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
          "id": "e19100c3ddb56b6fd6824dc6ef55ec580d076639",
          "message": "Merge pull request #198 from LoveDaisy/fix/gpu-color-mask-batch-leak\n\nfix(gpu): layer-0 color-class mask cross-batch leak (Metal + CUDA)",
          "timestamp": "2026-07-15T13:17:01+08:00",
          "tree_id": "c9be5f895a1ea8318033db8cb4029ccd31fb54f1",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e19100c3ddb56b6fd6824dc6ef55ec580d076639"
        },
        "date": 1784093081204,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 99.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 96.5,
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
          "id": "2cbcffd86aaaf5817d43b9b2504d063d705d802c",
          "message": "Merge pull request #199 from LoveDaisy/feat/painter-alpha-over\n\nfeat(painter-composite): painter 改亮度即 alpha 的 Porter-Duff over 合成 + 设默认",
          "timestamp": "2026-07-15T15:15:11+08:00",
          "tree_id": "8745cf535cc23d151ae929caa4af4ab41d872d10",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/2cbcffd86aaaf5817d43b9b2504d063d705d802c"
        },
        "date": 1784100158780,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 66.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu ARM64",
            "value": 100.4,
            "unit": "%"
          },
          {
            "name": "Ubuntu x86_64",
            "value": 95.7,
            "unit": "%"
          },
          {
            "name": "Windows MSVC x86_64",
            "value": 95.4,
            "unit": "%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "zhangjiajie043@gmail.com",
            "name": "Jiajie Zhang",
            "username": "LoveDaisy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e8f266452c9b204f5f067a0e9b7de29251e583e6",
          "message": "Merge pull request #200 from LoveDaisy/test/painter-default-e2e-coverage\n\ntest(painter-default-e2e): 补 painter 默认合成模式的 e2e 全链路覆盖",
          "timestamp": "2026-07-15T16:30:15+08:00",
          "tree_id": "e9f46c194ddee502cab098deb32fc54ec94eba1b",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/e8f266452c9b204f5f067a0e9b7de29251e583e6"
        },
        "date": 1784104660387,
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
            "value": 95.8,
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
          "id": "9e05cea90d94df75e7d0bf0cd62dce574afaffaa",
          "message": "Merge pull request #201 from LoveDaisy/chore/classify-pixels-stale-docstring\n\ndocs(image_utils): 修正 classify_pixels_by_color_direction 过时 docstring",
          "timestamp": "2026-07-15T17:49:19+08:00",
          "tree_id": "b15db229e3139294d532f532c945a31d5c0b255f",
          "url": "https://github.com/LoveDaisy/ice_halo_sim/commit/9e05cea90d94df75e7d0bf0cd62dce574afaffaa"
        },
        "date": 1784109422922,
        "tool": "customBiggerIsBetter",
        "benches": [
          {
            "name": "macOS ARM64",
            "value": 75.8,
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
      }
    ]
  }
}