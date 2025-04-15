import numpy as np

whole = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
new = whole
j = 0
while j < len(new) - 1:
    print(f"j = {j}, len(new) = {len(new)}, new[j] = {new[j]}")
    for i in range(j + 1, len(new)):
        # print(range(j + 1, len(new)))
        # print(f"i = {i}, len(new) = {len(new)}")
        if new[i] - new[j] > 0 or new[i] == 1:
            # whole = new[: j + 1] + new[i - 1 :]
            print(f"Move from {new[j]} to {new[i]} not valid.")
            break
        else:
            whole = new[: j + 1] + new[i:]
            print(f"Move from {new[j]} to {new[i]} valid.")
            # print(f"whole = {whole}")
    new = whole
    print(f"new = {new}")
    j += 1
print(f"j = {j}, len(new) = {len(new)}, new[j] = {new[j]}")
print(new)
