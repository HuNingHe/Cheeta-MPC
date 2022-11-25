#ifndef SHAREDMEMORY_H
#define SHAREDMEMORY_H

#include <iostream>
#include <string>
#include <fcntl.h>
#include <cassert>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <cerrno>

#if _WIN32
#include <windows.h>
#elif __linux
#include <sys/mman.h>
#include <sys/stat.h>
#endif

#include "ControlParameters.h"

#if _WIN32
/*!
 * A container class for an object which is stored in shared memory.  This
 * object can then be viewed in multiple processes or programs.  Note that there
 * is significant overhead when creating a shared memory object, so it is
 * recommended that two programs that communicate should have one single large
 * SharedMemoryObject instead of many small ones.
 *
 * A name string is used to identify shared objects across different programs
 *
 * Creating/deleting the memory can be done with createNew/closeNew.
 * Attention that when on linux platform, createNew and closeNew should be used in pairs, 
 * otherwise the mapped file will not be deleted automatically, the program believes that 
 * the shared memory has not been freed.
 *  
 * Viewing an existing object allocated with createNew can be done with
 * attach/detach
 */
template<typename T>
class SharedMemoryObject{
private:
    HANDLE _fileMapping;
    T* _data = nullptr;
	size_t _size;
    std::string _name;

public:
    SharedMemoryObject() = default;
    ~SharedMemoryObject(){};
    /*!
    * Allocate memory for the shared memory object and attach to it.
    * If allowOverwrite is true, and there's already an object with this name,
    * the old object is overwritten Note that if this happens, the object may be
    * initialized in a very weird state.
    */
    void createNew(const std::string& name, size_t size, bool allowOverwrite = false){
        // Size should be an integer multiple of 4096, this automatically done by system
		_size = size;
		if(name.length() == 0){
            throw std::runtime_error("[Shared Memory] SharedMemoryObject::createNew: Shared memory name is NULL string!");
		}
		_name = name;

        HANDLE fileHandle = CreateFile(name.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                       NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
		if(GetLastError() == ERROR_ALREADY_EXISTS){ // Check that if the file has already existed
			if (!allowOverwrite){
                throw std::runtime_error("[Shared Memory] SharedMemoryObject::createNew on something that wasn't new, file has already existed!");
			}
		}

        if (fileHandle == INVALID_HANDLE_VALUE){
            throw std::runtime_error("[ERROR] SharedMemoryObject create file failed: INVALID_HANDLE_VALUE");
        }

		printf("[Shared Memory] Open new %s, size %ld bytes\n", name.c_str(), size);

		_fileMapping = OpenFileMapping(PAGE_READWRITE, false, name.c_str());
		if (_fileMapping != NULL){
            // 避免同名但不同大小的共享内存, 因此此处禁止映射文件
            throw std::runtime_error("[Shared Memory] SharedMemoryObject::createNew(): fileMapping with the same name has already existed, please close it and retry");
		}

        _fileMapping = CreateFileMapping(fileHandle, NULL, PAGE_READWRITE, 0, size, name.c_str());
        if(_fileMapping == NULL){
            CloseHandle(fileHandle);
            throw std::runtime_error("[ERROR] SharedMemory::createNew() MapViewOfFile failed!");
        }

        void *shmBase = MapViewOfFile(_fileMapping, FILE_MAP_ALL_ACCESS, 0, 0, size);
        memset(shmBase, 0, size);
        _data = (T*)shmBase;
        CloseHandle(fileHandle);
        printf("[Shared Memory] SharedMemory create success(%s), map file to memory of size(%ld)\n", name.c_str(), size);

    }

    /*!
     * Attach to an existing shared memory object.
    */
    void attach(const std::string &name, size_t size){
        _size = size;
		if(name.length() == 0){
            throw std::runtime_error("[Shared Memory] SharedMemoryObject::attach: name is NULL string!");
		}
		_name = name;
        // attention that when using OpenFileMapping function, we must use FILE_MAP_ALL_ACCESS other than PAGE_READWRITE,otherwise it doesn't work
        _fileMapping = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, name.c_str());
        if (_fileMapping == NULL){
            throw std::runtime_error("[Shared Memory] SharedMemory attach failed");
        }
        // attention that MapViewOfFile treat FILE_MAP_ALL_ACCESS as FILE_MAP_WRITE
        void *shmBase = MapViewOfFile(_fileMapping, FILE_MAP_ALL_ACCESS, 0, 0, size);
        _data = (T*)shmBase;
        printf("[Shared Memory] SharedMemory attach success(%s), map file to memory of size(%ld)\n", name.c_str(), size);
    }
    /*!
     * Free memory associated with the current open shared memory object.  The
     * object could have been opened with either attach or createNew.  After
     * calling this, no process can use this shared object
     */
    void closeNew(){
		if(_data == NULL){
            throw std::runtime_error("[ERROR] SharedMemoryObject::closeNew(): the shared memory doesn't exist");
		}
		if(!UnmapViewOfFile((void *)_data)){
            throw std::runtime_error("[ERROR] SharedMemoryObject::closeNew(): UnmapViewOfFile failed");
        }

		if (!CloseHandle(_fileMapping)){
            throw std::runtime_error("[ERROR] SharedMemoryObject::closeNew() Close fileMapping error");
		}

		_data = nullptr;
		_fileMapping = nullptr;

		HANDLE fileHandle = CreateFile(_name.c_str(), DELETE, 0, NULL, OPEN_EXISTING, FILE_FLAG_DELETE_ON_CLOSE, NULL);
		if(fileHandle == INVALID_HANDLE_VALUE){
            printf("[ERROR] SharedMemoryObject::closeNew(%s) failed: delete file error when close shared memory, file may not exist or be occupied\n", _name.c_str());
		}
		CloseHandle(fileHandle);
		fileHandle = INVALID_HANDLE_VALUE;
		printf("[Shared Memory] SharedMemoryObject::closeNew (%s) success\n", _name.c_str());
    }

    /*!
     * Close this view of the currently opened shared memory object. The object
     * can be opened with either attach or createNew.  After calling this, this
     * process can no longer use this shared object, but other processes still can.
   */
    void detach(){
		if(_data == NULL){
			throw std::runtime_error("[ERROR] SharedMemoryObject::detach() failed, the shared memory doesn't exist");
		}

		if(!UnmapViewOfFile((void *)_data)){
            throw std::runtime_error("[ERROR] SharedMemoryObject::detach(): UnmapViewOfFile failed");
        }
        printf("[Shared Memory] SharedMemoryObject::detach (%s) success\n", _name.c_str());
        _data = nullptr;
    }

	void writeData(void *dataSrc){
        memcpy(_data, dataSrc, _size);
	}

	void readData(void *dataDst){
        memcpy(dataDst, _data, _size);
	}
	
    T* get(){
		assert(_data);
		return _data;
    }

    T& operator()(){
        assert(_data);
		return *_data;
    }
};

#elif __linux
template <typename T>
class SharedMemoryObject{
public:
  	T* _data = nullptr;
  	int _fd;
	size_t _size;
	std::string _name;

public:
	SharedMemoryObject() = default;

  	void createNew(const std::string& name, size_t size, bool allowOverwrite = false) {
		_size = size;
		if(name.length() == 0){
            throw std::runtime_error("[Shared Memory] SharedMemoryObject::createNew: Shared memory name is NULL string!");
		}
		_name = name;
		struct stat s{}; // restore file information

		_fd = shm_open(name.c_str(), O_RDWR | O_CREAT, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH);
		if(_fd < 0){
            throw std::runtime_error("[ERROR] SharedMemoryObject create file failed");
		}

		printf("[Shared Memory] Open new %s, size %ld bytes\n", name.c_str(), size);

		if (fstat(_fd, &s)){
            throw std::runtime_error("[ERROR] SharedMemoryObject file state error");
		}

		if(s.st_size){
			if (!allowOverwrite){
                throw std::runtime_error("[Shared Memory] SharedMemoryObject::createNew on something that wasn't new, file has already existed!");
			}
		}

		if (ftruncate(_fd, size)){
            throw std::runtime_error("[ERROR] SharedMemoryObject::createNew(): ftruncate() error");
		}

		void* mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
		if (mem == MAP_FAILED) {
            throw std::runtime_error("[ERROR] SharedMemory::createNew() mmap failed!");
		}
        memset(mem, 0, size);
        _data = (T*)mem;
        printf("[Shared Memory] SharedMemory create success(%s), map file to memory of size(%ld)\n", name.c_str(), size);
	}

  	void attach(const std::string& name, size_t size){
		_size = size;
		if(name.length() == 0){
            throw std::runtime_error("[Shared Memory] SharedMemoryObject::createNew: Shared memory name is NULL string!");
		}
		_name = name;
		struct stat s{};

		_fd = shm_open(name.c_str(), O_RDWR, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IROTH);
		if (_fd < 0){
            throw std::runtime_error("[ERROR] SharedMemoryObject::attach(): open file failed");
        }

		printf("[Shared Memory] open existing %s size %ld bytes\n", name.c_str(), size);

		if (fstat(_fd, &s)){
            throw std::runtime_error("[ERROR] SharedMemoryObject file state error");
		}

		if (s.st_size != size){
            throw std::runtime_error("[Shared Memory] SharedMemoryObject::attach on incorrect size!");
		}

		void* mem = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
		if (mem == MAP_FAILED){
            throw std::runtime_error("[ERROR] SharedMemory::attach(): mmap failed!");
		}
        _data = (T*)mem;
        printf("[Shared Memory] SharedMemory attach success(%s), attached memory of size(%ld)\n", name.c_str(), size);
  	}

	void closeNew(){
		if(_data == NULL){
            throw std::runtime_error("[ERROR] SharedMemoryObject::closeNew(): the shared memory doesn't exist");
		}

		if (munmap((void*)_data, _size)){
            throw std::runtime_error("[ERROR] SharedMemoryObject::closeNew(): munmap failed");
        }


		if (shm_unlink(_name.c_str())){
            throw std::runtime_error("[ERROR] SharedMemoryObject::closeNew() shm_unlink error");
		}

		// close fd and delete shared file
		if (close(_fd)){
			printf("[ERROR] SharedMemoryObject::closeNew (%s) close %s\n", _name.c_str(), strerror(errno));
		}

		_fd = 0;
        _data = nullptr;
		printf("[Shared Memory] Shared memory %s closed. if you want to use it again, please create a new one.\n", _name.c_str());
	}

	void detach(){
		if(_data == NULL){
            throw std::runtime_error("[ERROR] SharedMemoryObject::detach() failed, the shared memory doesn't exist");
        }

		if (munmap((void*)_data, _size)){
            throw std::runtime_error("[ERROR] SharedMemoryObject::detach(): munmap failed");
		}

		_data = nullptr;

		if(close(_fd)){
			printf("[ERROR] SharedMemoryObject::closeNew (%s) close %s\n", _name.c_str(), strerror(errno));
		}
		_fd = 0;
		printf("[Shared Memory] Shared memory %s detached. if you want to use it again, please attach again.\n", _name.c_str());
    }

	void writeData(void *dataSrc){
        memcpy(_data, dataSrc, _size);
	}

	void readData(void *dataDst){
        memcpy(dataDst, _data, _size);
	}

	T* get(){
		assert(_data);
		return _data;
  	}

	T& operator()(){
		assert(_data);
		return *_data;
	}
};

#endif
#endif