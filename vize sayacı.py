from datetime import datetime

print("--- VIZE/FINAL GERI SAYIM ARACI ---")
print("Ornek format: 25.01.2025 14:00")

try:
    tarih_girisi = input("Sinav tarihini ve saatini giriniz (GG.AA.YYYY SS:DD): ")

    sinav_tarihi = datetime.strptime(tarih_girisi, "%d.%m.%Y %H:%M")
    su_an = datetime.now()

    if sinav_tarihi < su_an:
        print("\nBu sinav tarihi gecmis.")
    else:
        fark = sinav_tarihi - su_an
        gun = fark.days
        saniye = fark.seconds
        saat = saniye // 3600
        dakika = (saniye % 3600) // 60

        print("\nSinava kalan sure:")
        print(f"{gun} Gun, {saat} Saat, {dakika} Dakika")

except ValueError:
    print("\nHata: Lutfen tarihi tam olarak 'GG.AA.YYYY SS:DD' formatinda giriniz.")