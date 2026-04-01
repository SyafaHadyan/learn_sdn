# Shortest Path forwarding
Berisikan implementasi shortest-path forwarding di OSKen (fork dari Ryu).

Implementasi saat ini menggunakan Mode 1:
- OSKen topology discovery (`--observe-links`) menangani LLDP dan link-state.
- Controller SPF hanya membaca topologi dari `get_link()`/`get_switch()`.
- Flow SPF diberi cookie khusus agar dapat dihapus dan dipasang ulang saat topologi berubah.
- Packet yang belum punya tujuan dikenal akan dibroadcast lewat spanning broadcast tree yang dihitung oleh controller, bukan via flood bebas dari switch.

1. Jalankan mininet dengan program dengan perintah pada salah satu terminal console:
```
sudo python3 topo-spf_lab.py
```
  Topologi Mininet ini memakai `autoSetMacs=True` (MAC host/switch terisi otomatis dan konsisten) dan `autoStaticArp=True` (setiap host otomatis punya ARP statis ke host lain) agar eksperimen fokus ke perilaku SPF, bukan isu alamat/MAC/ARP.
2. Jalankan controller langsung dengan Python pada terminal console lainnya:
```
python3 SPF/dijkstra_osken_controller.py
```
  Controller ini sudah mengaktifkan topology discovery internal OSKen, jadi tidak perlu menjalankan `osken-manager` secara manual.

### Opsi Controller Multipath (ECMP)

Selain controller single-path, tersedia dua controller multipath baru:

- Dijkstra multipath (equal-cost multipath dari parent-set Dijkstra):
  ```
  python3 SPF/dijkstra_multipath_osken_controller.py
  ```

- A* multipath (equal-cost multipath dari parent-set A*):
  ```
  python3 SPF/astar_multipath_osken_controller.py
  ```

Keduanya memasang OpenFlow SELECT group pada ingress switch untuk load-sharing lintasan berbiaya sama.

### Mode Menjalankan Controller (Normal vs Verbose)

- Mode normal (cukup untuk praktikum dasar):
  ```
  python3 SPF/dijkstra_osken_controller.py
  ```
- Mode verbose (untuk melihat detail event dan debug path):
  ```
  python3 SPF/dijkstra_osken_controller.py --verbose
  ```
- Mode debug level eksplisit (lebih detail dari normal):
  ```
  python3 SPF/dijkstra_osken_controller.py --default-log-level 10
  ```

Catatan:
- `--default-log-level 20` = INFO (default yang lebih ringkas).
- `--default-log-level 10` = DEBUG (lebih ramai, cocok untuk analisis).
- Saat controller dihentikan dengan `Ctrl+C`, exit code `130` adalah normal.

Pada program `topo-spf_lab.py` akan membentuk topologi yang terdapat loopnya seperti tampak pada gambar topologi yang terdiri atas 6 hosts (h1 - h6) dan 3 switch (s1, s2, s3) dengan skenario sebagai berikut:
![topology](/SPF/img/3sw6h_loop.jpg)
- semua host (h1 - h6) harus dapat terhubung satu sama lain dan menggunakan jalur terpendek
  ```
  mininet> h1 ping h6
  ```
- ujicoba pemutusan salah satu jalur, semisal
  ```
  mininet> link s1 s3 down
  mininet> h1 ping h6
  ```
Pada saat ping sebelum dan setelah `link s1 s3 down` amati path terpilih pada terminal yang menjalankan aplikasi dijkstra `python3 SPF/dijkstra_osken_controller.py`.
Controller akan menghapus flow SPF lama dan memasang ulang flow baru ketika topologi berubah.
Lakukan verifikasi pada tabel flow pada switch terkait apa saja yang berubah:
```
mininet> dpctl dump-flows -O OpenFlow13
```
Lakukan beberapa ujicoba dengan kombinasi host lainnya atau pun pemutusan jalur lainnya, dan amati apa yang tampil pada terminal console yang menjalankan `python3 SPF/dijkstra_osken_controller.py` atau pun pada terminal console yang menjalankan `python3 SPF/topo-spf_lab.py`.

### Verifikasi Singkat
1. Jalankan `h1 ping h6` dan pastikan ping berhasil setelah learning awal.
2. Jalankan `link s1 s3 down`, lalu ulangi `h1 ping h6` dan pastikan trafik tetap jalan lewat jalur alternatif.
3. Jalankan `link s1 s3 up`, lalu ulangi `h1 ping h6` untuk memastikan jalur kembali ke rute terpendek.
4. Jika ingin melihat flow, jalankan `dpctl dump-flows -O OpenFlow13` pada switch yang relevan.

### Membaca Log dengan Cepat

- `[HOST-LEARN]`: host baru terdeteksi atau berpindah port.
- `[TOPO-CHANGE]`: ada perubahan link (up/down).
- `[TOPO] refreshed topology`: controller selesai membaca snapshot topologi baru.
- `[PKT-FWD]`: packet mendapatkan path dan diteruskan.
- `[PKT-DROP]`: packet dibuang karena kondisi tertentu (misalnya belum ada path).

Interpretasi praktis:
- Jika setelah `link down` muncul `[TOPO-CHANGE]` lalu ping kembali normal, berarti convergence berjalan baik.
- Jika drop muncul terus menerus, cek kembali topologi Mininet, status link, dan mode log DEBUG untuk detail.

### Troubleshooting Singkat

1. Ping awal gagal di percobaan pertama:
  - Ulangi ping 2-3 kali, karena learning + flow install butuh warm-up.
2. Tidak ada log topology change saat `link down/up`:
  - Pastikan controller dijalankan dari file ini dan Mininet terhubung ke remote controller.
3. Log terlalu ramai:
  - Gunakan mode normal tanpa `--verbose`.
4. Ingin diagnosa detail:
  - Gunakan `--default-log-level 10`, lalu ulangi skenario `link down/up`.

### Catatan Implementasi
- LLDP dipakai oleh OSKen topology discovery untuk membangun link-state.
- PacketIn dari LLDP tidak diproses oleh controller SPF karena itu bagian dari discovery, bukan forwarding data-plane.
- Implementasi ini masih Mode 1: belum self-contained LLDP sender/receiver di aplikasi SPF.

### Roadmap
Rencana pengembangan lanjutan didokumentasikan di `SPF/PLAN.md`.

