def getModelFileType(path):
    if path.endswith('.sdf'):
        return "sdf"
    elif path.endswith('.urdf'):
        return "urdf"
    else:
        raise TypeError('the file must be .sdf or .urdf')


def getParserArgsRobot():
    import argparse
    parser = argparse.ArgumentParser(description='Robot environment argument provider.')
    parser.add_argument('-g', '--gzclient', action='store_true', help='Run user interface.')
    parser.add_argument('-d', '--debug', action='store_true', help='Execute in debug mode. \
        Need to launch all the processes elsewhere.', default=True)
    parser.add_argument('-r', '--realSpeed', action='store_true', help='Execute the simulation in \
        real speed and using the running specific driver.', default=True)
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('-m', '--multiInstance', action='store_true', help='Provide network \
        segmentation to allow multiple instances.')
    group.add_argument('-p', '--port', type=int, default=11345, help='Provide exact port to the \
        network segmentation to allow multiple instances.')

    return parser
