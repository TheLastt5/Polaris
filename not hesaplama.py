try:
    alinan_not = int(input("Lütfen sınav notunuzu (0-100 arasında) girin: "))
except ValueError:
    print("Hata: Lütfen sadece tam sayı girin.")
    exit()

if alinan_not < 0 or alinan_not > 100:
    print("Geçersiz not girdiniz. Not 0 ile 100 arasında olmalı.")

elif alinan_not >= 50:
    print(f"Notunuz: {alinan_not} TEBRİKLER GEÇTİNİZ")

else:
    print(f"Notunuz: {alinan_not} MAALESEF KALDINIZ")