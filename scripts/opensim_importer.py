import os


def readMotionFile(filename):
    """ Reads OpenSim .sto files.

    Parameters
    ----------

    filename: absolute path to the .sto file

    Returns
    -------

    header: the header of the .sto
    labels: the labels of the columns
    data: an array of the data

    """

    if not os.path.exists(filename):
        print('file do not exists')

    file_id = open(filename, 'r')

    # read header
    next_line = file_id.readline()
    header = [next_line]
    nc = 0
    nr = 0
    while not 'endheader' in next_line:
        if 'datacolumns' in next_line:
            nc = int(next_line[next_line.index(' ') + 1:len(next_line)])
        elif 'datarows' in next_line:
            nr = int(next_line[next_line.index(' ') + 1:len(next_line)])
        elif 'nColumns' in next_line:
            nc = int(next_line[next_line.index('=') + 1:len(next_line)])
        elif 'nRows' in next_line:
            nr = int(next_line[next_line.index('=') + 1:len(next_line)])
        elif 'version=2' in next_line:
            raise Exception('Importer do not support version=2 format yet.')

        next_line = file_id.readline()
        header.append(next_line)

    # process column labels
    next_line = file_id.readline()
    if next_line.isspace() == True:
        next_line = file_id.readline()

    labels = next_line.split()

    # get data
    data = []
    for i in range(1, nr + 1):
        d = [float(x) for x in file_id.readline().split()]
        data.append(d)

    file_id.close()

    return header, labels, data


def index_containing_substring(list_str, substring):
    """For a given list of strings finds the index of the element that contains the
    substring.

    Parameters
    ----------

    list_str: list of strings
    substring: substring

    Returns
    -------

    index: containing the substring or -1

    """
    for i, s in enumerate(list_str):
        if substring in s:
            return i
    return -1


def indices_containing_substring(list_str, substring):
    """For a given list of strings finds the indices containing the substring.

    Parameters
    ----------

    list_str: list of strings
    substring: substring

    Returns
    -------

    index: containing the substring or -1

    """
    indices = []
    for i, s in enumerate(list_str):
        if substring in s:
            indices.append(i)
    return indices
