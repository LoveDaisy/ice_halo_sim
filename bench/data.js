window.BENCHMARK_DATA = {
  "lastUpdate": 1775448149305,
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
      }
    ]
  }
}