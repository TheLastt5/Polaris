import re

def sifre_kontrol(sifre):
    """
    Şifrenin güçlü olup olmadığını kontrol eder.
    Kurallar:
    1. En az 1 Büyük Harf
    2. En az 1 Sayı
    3. En az 1 Özel Karakter
    """

    pattern = r"^(?=.*[A-Z])(?=.*\d)(?=.*[^a-zA-Z0-9]).{8,}$"
    
    if re.match(pattern, sifre):
        return True
    else:
        return False

if __name__ == "__main__":
    print("---GÜVENLİ SİSTEMİ ---")
    kullanici_sifresi = input("Lütfen yeni şifrenizi belirleyin: ")
    
    if sifre_kontrol(kullanici_sifresi):
        print("ŞİFRE GÜÇLÜ: Onaylandı.")
    else:
        print("ŞİFRE ZAYIF: Lütfen kurallara uyunuz!")
        print("   - En az 1 Büyük Harf")
        print("   - En az 1 Sayı")
        print("   - En az 1 Özel Karakter (!, ?, @, # vb.)")