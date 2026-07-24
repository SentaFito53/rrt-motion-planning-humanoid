# motplan — RRT Motion Planning Test (BarelangFC)

## Tujuan

Repository/modul ini merupakan **mini project pengujian (proof of concept)** implementasi algoritma **RRT (Rapidly-exploring Random Tree)** untuk motion planning, yang diintegrasikan ke dalam framework kontrol robot humanoid sepak bola **BarelangFC** (berbasis ROS2 dan BehaviorTree.CPP).

Tujuan utamanya **bukan** untuk deployment fitur produksi, melainkan untuk:
- Menguji kelayakan algoritma RRT dalam menghasilkan jalur bebas obstacle pada skala lapangan robot soccer.
- Memverifikasi integrasi algoritma path planning dengan sistem kontrol gerak (motion primitive) dan odometri robot yang sudah ada.
- Menjadi dasar eksperimen lanjutan sebelum algoritma sejenis dipakai untuk kebutuhan nyata seperti penghindaran robot lawan/rekan tim saat pertandingan.

## Struktur Komponen

| File | Peran |
|---|---|
| `rrt_planner.hpp` | Implementasi murni algoritma RRT (tidak bergantung ROS) |
| `main.cpp` (node `main_strategy`) | Integrasi RRT ke behavior tree robot, lewat node `testGrid` |
| `rrt_monitor.py` | Node ROS2 terpisah untuk visualisasi jalur, obstacle, dan posisi robot di peta |

## Fungsi `NodeStatus testGrid()`

`testGrid()` adalah node **Behavior Tree** yang berfungsi sebagai *state machine* pengendali siklus motion planning RRT, dengan tiga status (`PlanState`):

1. **`IDLE`** — Dijalankan sekali di awal. Mendefinisikan titik start `(0,0)`, titik goal `(300,0)`, dan obstacle lokal secara manual (posisi `x, y, radius` dalam cm). Memanggil `RRTPlanner::plan()` untuk menghasilkan jalur, lalu `smoothPath()` untuk menghaluskannya. Jika jalur ditemukan, status berpindah ke `EXECUTING`; jika gagal, node mengembalikan `NodeStatus::FAILURE`.
2. **`EXECUTING`** — Mengeksekusi jalur waypoint demi waypoint menggunakan primitif gerak `new_out_pos_norotate()` (closed-loop walk-to-point berbasis feedback odometri, tanpa memaksa align yaw robot di tiap waypoint). Setelah satu waypoint tercapai (`doneMoved == true`), index waypoint bertambah dan lanjut ke waypoint berikutnya.
3. **`DONE`** — Dipanggil setelah seluruh waypoint tercapai. Robot dihentikan (`motion("0")`), lalu status dan obstacle di-reset ke kondisi awal (`IDLE`, `localObstacles.clear()`) agar node dapat diuji ulang dari nol, dan mengembalikan `NodeStatus::SUCCESS`.

Selama proses `IDLE` dan `EXECUTING` berlangsung, node selalu mengembalikan `NodeStatus::FAILURE` agar Behavior Tree tetap menganggap node ini "berjalan" (running) sampai benar-benar mencapai `DONE`. Di setiap tick, `publishRRTVisualization()` dipanggil untuk mempublikasikan jalur dan obstacle terbaru ke topic ROS2, sehingga proses planning dapat dipantau secara real-time.

## Algoritma RRT yang Digunakan

Implementasi RRT (`rrt_planner.hpp`) berupa **RRT dasar dengan goal bias**, terdiri dari:

- **`RRTPlanner::plan(start, goal, obstacles)`** — Fungsi utama. Menumbuhkan pohon eksplorasi dari titik start dengan sampling acak (10% kemungkinan langsung sampling ke goal untuk mempercepat konvergensi), lalu melakukan *steering* menuju titik sampel sejauh maksimal `stepSize` (20 cm). Setiap segmen baru diperiksa terhadap tabrakan obstacle (`segmentCircleIntersect`) sebelum ditambahkan ke pohon. Berhenti begitu satu node cukup dekat dengan goal (`goalTolerance` 15 cm) dan segmen ke goal bebas obstacle.
- **`smoothPath(path, obstacles)`** — Fungsi pasca-proses opsional yang menghapus waypoint perantara yang bisa "dilompati" langsung tanpa menabrak obstacle, menghasilkan jalur lebih pendek dan tidak zig-zag.
- **Obstacle** direpresentasikan sederhana sebagai lingkaran `{x, y, radius}`, dengan radius mencakup ukuran fisik robot lawan ditambah margin keamanan.

## Integrasi dengan Framework ROS2

```
main_strategy (C++, node BehaviorTree.CPP)
    │
    ├─ testGrid() [state machine IDLE → EXECUTING → DONE]
    │     ├─ RRTPlanner::plan()      → hasil: vector<Point2D> rrtPath
    │     └─ new_out_pos_norotate()  → closed-loop walk-to-point per waypoint
    │
    ├─ publish topic 'rrt_path'       (nav_msgs/Path)
    ├─ publish topic 'rrt_obstacles'  (std_msgs/Float32MultiArray, format flat [x,y,r,...])
    └─ publish topic 'pose'           (nav_msgs/Odometry, dari sistem odometri robot)
              │
              ▼
rrt_monitor.py (Python, node terpisah)
    ├─ subscribe 'pose', 'rrt_path', 'rrt_obstacles'
    └─ render peta lapangan + jalur + obstacle + posisi robot via OpenCV
```

- **`rrt_planner.hpp`** murni C++ standar tanpa dependency ROS, sehingga logika algoritma dapat diuji dan dikembangkan secara independen dari sistem robot.
- **`main_strategy`** menjembatani hasil planning dengan sistem gerak robot yang sudah ada, memanfaatkan data odometri (`robotPos_X/Y`, `msg_yaw`) sebagai feedback posisi closed-loop.
- **`rrt_monitor`** berjalan sebagai node ROS2 terpisah (dapat dijalankan di komputer lain dalam jaringan ROS2 yang sama), berguna untuk debugging visual tanpa membebani komputasi node strategi utama robot.
- Robot dapat memiliki namespace (`/robot_<id>/...`) sehingga `rrt_monitor` mendukung parameter `robot_id` untuk memilih robot mana yang ingin dipantau.

## Status Project

Mini project ini masih bersifat **eksperimental/testing**, dengan obstacle yang didefinisikan secara manual (hardcoded) di dalam `testGrid()`. Pengembangan lanjutan yang mungkin diperlukan antara lain: pengambilan posisi obstacle secara otomatis dari deteksi visual atau data koordinasi antar robot, serta pengujian pada skenario lapangan yang lebih kompleks.
