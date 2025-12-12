sayi = int(input("Bir sayı girin (N): "))

toplam = 0

for i in range(1, sayi + 1):
    toplam += i

print(f"1'den {sayi}'e kadar olan sayıların toplamı: {toplam}")