def print_formatted_dict_list(dict_list):
    for item in dict_list:
        formatted_items = []
        for key, value in item.items():
            if isinstance(value, float):
                formatted_items.append(f"'{key}': {value:.3f}")
            else:
                formatted_items.append(f"'{key}': {value}")
        print("{ " + ", ".join(formatted_items) + " }\n")


