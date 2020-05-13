
aaa = 0

# *********************************************************************
def Convert_Str_to_Bytearray(text_in):
    '''
    :param text_in:
    :param bytearray_out:
    :return:
    '''
    if isinstance(text_in, str):
        bytearray_out = bytearray(b'')
        for data in text_in:
            bytearray_out += ord(data).to_bytes(1, 'big')
        return bytearray_out
    else:
        return None


def Convert_ArrBite_to_ArrChar(data, data_from, size, size_max):
    '''
    #*********************************************************************
    # извлечение номера в формате str из данных в формате byte
    # [data] - данные в формате byte
    # [data_from] - индекс начала для обработки
    # [size] - длина данных для обработки
    # [size_max] - максимально допустимая длина данных
    #*********************************************************************
    '''
    if (size > 0) and (size < size_max):
        data_to = size - 2
        text = ''
        for i in range(data_from, data_to):
            text += chr(data[i])
        return text
