def main():
    for bump in range(128):
        if bump < 64:
            bank_group = 0
            bank_index = 0
        else:
            bank_group = 8
            bank_index = 64

        h_index = bump
        v_group, v_group_index = (h_index-bank_group) % 8, (h_index - bank_index) // 8
        v_index = v_group * 8 + v_group_index + bank_index
        t_index = v_index
        t_group, t_group_index = t_index % 8, (t_index-bank_index) // 8
        print(f"h_group: {h_index // 8}, h_group_index: {h_index % 8}")
        print(f"t_group: {t_group}, t_group_index: {t_group_index}")
        print("--------------------------------")


if __name__ == "__main__":
    main()
