while True:
    try:
        veri = input("Bir sayı giriniz: ")
        sayi = int(veri)

        print(f"Teşekkürler, girdiğiniz sayı: {sayi}")
        break 
        
    except ValueError:
        print("Lütfen geçerli bir sayı giriniz.")