# maps a variables values based on old range and new range linearly
# source: https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
def variable_mapping(value, from_low, from_high, to_low, to_high):
    """
    maps a variables values based on old range and new range linearly
    source: https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio

    :param value: the desired value to map to the range
    :param from_low: the previous range low
    :param from_high: the previous range high
    :param to_low: the desired range low
    :param to_high: the desired range high
    :return:
    """
    new_range = (to_high - to_low)
    old_range = (from_high - from_low)
    new_value = (((value - from_low) * new_range) / old_range) + to_low
    return new_value
