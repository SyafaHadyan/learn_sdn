# Server Load Balancing
Berisikan eksplorasi tentang aplikasi server load balancer di OSKen (fork dari Ryu). Topologi mengacu pada gambar berikut:
![img](/LB/img/topo_lb.jpg)

1. Jalankan mininet dengan program dengan perintah pada salah satu terminal console:
```
python3 topo_lb.py
```
Catatan: jika tidak menjalankan environment sebagai `root` (mis. di host tanpa container privileged), gunakan `sudo`.
2. Jalankan aplikasi `rr_lb.py` langsung dengan Python pada terminal console lainnya:
```
python3 LB/rr_lb.py
```

Pada program `topo_lb.py` akan membentuk topologi seperti ![topology] yang terdiri atas 4 hosts (h1 - h4) dan 1 switch (s1) dengan skenario sebagai berikut:
- h1 sebagai web client
- h2 sebagai web server `mininet> h2 python3 -m http.server 80 &`
- h3 sebagai web server `mininet> h3 python3 -m http.server 80 &`
- h4 sebagai web server `mininet> h4 python3 -m http.server 80 &`

Pada program `rr_lb.py` secara umum akan mengeksekusi
- Virtual server pada IP 10.0.0.100
- Menentukan h2, h3, dan h4 sebagai server tujuan
- Menggunakan round-robin untuk meneruskan setiap tcp request baru ke server yang dipilih

Selanjutnya lakukan percobaan pada mininet console:
```
mininet> h1 curl 10.0.0.100
mininet> dpctl dump-flows -O OpenFlow13
```
Lakukan beberapa kali dan amati apa yang tampil pada terminal console yang menjalankan `python3 LB/rr_lb.py` atau pun pada terminal console yang menjalankan Mininet.

(Di devcontainer biasanya sudah `root`, jadi cukup `python3 LB/topo_lb.py`.)

