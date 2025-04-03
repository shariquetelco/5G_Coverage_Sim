def calculate_link_budget(transmitter_power, transmitter_gain, receiver_gain, path_loss, receiver_sensitivity):
    """
    Calculates the link budget.
    Formula: Link Budget (dB) = Pt + Gt + Gr - L - Pr
    :param transmitter_power: Transmitter power (dBm)
    :param transmitter_gain: Transmitter gain (dB)
    :param receiver_gain: Receiver gain (dB)
    :param path_loss: Path loss (dB)
    :param receiver_sensitivity: Receiver sensitivity (dBm)
    :return: Link budget in dB
    """
    link_budget = transmitter_power + transmitter_gain + receiver_gain - path_loss - receiver_sensitivity
    return link_budget