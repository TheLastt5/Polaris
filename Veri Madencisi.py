import re

def veri_madencisi_lvl1():
    giris_dosyasi = "lvl1_bozuk_veri.txt"
    cikis_dosyasi = "lvl1_temiz_rehber.txt"
    email_pattern = r'[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}'
    tr_phone = r'(?:\+90|0)\s*\(?\d{3}\)?[\s.-]?\d{3}[\s.-]?\d{2}[\s.-]?\d{2}'
    us_phone = r'\+1\s*\(?\d{3}\)?[\s.-]?\d{3}[\s.-]?\d{4}'
    phone_pattern = f'{tr_phone}|{us_phone}'

    emails = []
    phones = []

    try:
        with open(giris_dosyasi, 'r', encoding='utf-8') as file:
            content = file.read()

            emails = re.findall(email_pattern, content)
            raw_phones = re.findall(phone_pattern, content)
            for p in raw_phones:
                if isinstance(p, tuple):
                    phones.append("".join(p))
                else:
                    phones.append(p)

    except FileNotFoundError:
        print(f"Hata: '{giris_dosyasi}' dosyası bulunamadı. Lütfen dosyayı oluşturduğundan emin ol.")
        return

    # --- 3. Temiz Dosyayı Yazma ---
    with open(cikis_dosyasi, 'w', encoding='utf-8') as file:
        file.write("--- E-POSTA LISTESI ---\n")
        for email in emails:
            file.write(email + "\n")
        
        file.write("\n--- TELEFON LISTESI ---\n")
        for phone in phones:
            file.write(phone + "\n")

    print(f"İşlem Tamam Kaptan!\n {len(emails)} e-posta ve {len(phones)} telefon numarası '{cikis_dosyasi}' dosyasına ayıklandı.")

if __name__ == "__main__":
    veri_madencisi_lvl1()