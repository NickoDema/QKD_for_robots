#include <fstream>
#include <vector>

/*
 * Выполняет операцию "исключающее ИЛИ" над пересылаемыми данными.
 * Для расшифрования необходимо выполнить XOR над принятыми данными и идентичным ключом.
 * @param   _data   пересылаемые данные
 * @param   _fn     Имя файла ключа в бинарном виде
 * @return  зашифрованные данные
 */
std::vector<unsigned char> XOR(std::vector<unsigned char> _data, char * _fn)
{
    //TODO: сделать проверку _data.size <= _fn.size
    //TODO: использовать файл ключа по кругу - для этого использовать static-переменную текущей позиции

    using namespace std;

    vector<unsigned char> key;
    {
        ifstream _if(_fn, ios::binary);
        _if.seekg(0, ios::end);
        size_t fsize = _if.tellg();
        _if.seekg(0, ios::beg);
        
        key.resize(fsize);
        _if.read((char*)&key[0], fsize);
    }

    vector<unsigned char> ret(_data.size());
    for (size_t i = 0; i < _data.size(); ++i)
    {
        // Мякотка
        ret[i] = _data[i]^key[i];
    }
    return ret;
}