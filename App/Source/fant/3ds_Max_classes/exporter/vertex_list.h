
#pragma once

#include <Max.h>
#include <cassert>
#include <cstdint>
#include <memory>

namespace fant {
class vertex_list {
public:
  using size_type = std::uint32_t;

  using map_channel_size_type = std::uint32_t;

  class vertex_attribute_viewer;

  class const_vertex_attribute_viewer {
    friend class vertex_attribute_viewer;

  public:
    const_vertex_attribute_viewer(const vertex_list &list_, size_type index_)
        : _list(const_cast<vertex_list &>(list_)), _index(index_) {
      assert(index_ < list_._nVertices);
    }

    const Point3 &vertex() const {
      return _list._vertexChannel[_index];
    }

    const Point3 &normal() const {
      return _list._normalChannel[_index];
    }

    const Point3 &texcoord() const {
      return _list._texcoordChannel[_index];
    }

    const Point3 &uvw(map_channel_size_type map_index_) const {
      return _list._mapChannels[map_index_][_index];
    }

    bool operator==(const const_vertex_attribute_viewer &other_) const {
      assert(_list.map_channel_size() == other_._list.map_channel_size());
      if (vertex() != other_.vertex() || normal() != other_.normal() ||
          texcoord() != other_.texcoord()) {
        return false;
      }
      for (map_channel_size_type iMap = 0; iMap < _list.map_channel_size();
           ++iMap) {
        if (uvw(iMap) != other_.uvw(iMap)) {
          return false;
        }
      }
      return true;
    }

  private:
    size_type _index;
    vertex_list &_list;
  };

  class vertex_attribute_viewer : public const_vertex_attribute_viewer {
  public:
    vertex_attribute_viewer(vertex_list &list_, size_type index_)
        : const_vertex_attribute_viewer(list_, index_) {
    }

    void vertex(const Point3 &value_) {
      _list._vertexChannel[_index] = value_;
    }

    void normal(const Point3 &value_) {
      _list._normalChannel[_index] = value_;
    }

    void texcoord(const Point3 &value_) {
      _list._texcoordChannel[_index] = value_;
    }

    void uvw(map_channel_size_type map_index_, const Point3 &value_) {
      _list._mapChannels[map_index_][_index] = value_;
    }

    vertex_attribute_viewer &
    operator=(const const_vertex_attribute_viewer &other_) {
      assert(_list.map_channel_size() == other_._list.map_channel_size());
      vertex(other_.vertex());
      normal(other_.normal());
      texcoord(other_.texcoord());
      for (map_channel_size_type iMap = 0; iMap < _list.map_channel_size();
           ++iMap) {
        uvw(iMap, other_.uvw(iMap));
      }
      return *this;
    }
  };

  vertex_list(size_type size_, map_channel_size_type map_channel_count_)
      : _nVertices(size_), _nMaps(map_channel_count_) {
    _vertexChannel = std::make_unique<Point3[]>(_nVertices);
    _normalChannel = std::make_unique<Point3[]>(_nVertices);
    _texcoordChannel = std::make_unique<Point3[]>(_nVertices);

    _mapChannels = std::make_unique<std::unique_ptr<Point3[]>[]>(_nMaps);
    for (map_channel_size_type iMapChannel = 0;
         iMapChannel < map_channel_count_; ++iMapChannel) {
      _mapChannels[iMapChannel] = std::make_unique<Point3[]>(_nVertices);
    }
  }

  size_type size() const {
    return _nVertices;
  }

  size_type map_channel_size() const {
    return _nMaps;
  }

  const_vertex_attribute_viewer operator[](size_type index_) const {
    return {*this, index_};
  }

  vertex_attribute_viewer operator[](size_type index_) {
    return {*this, index_};
  }

  std::pair<vertex_list, std::unique_ptr<size_type[]>> to_indexed() const {
    vertex_list resultVertexList{_nVertices, _nMaps};
    size_type nResultVertices = 0;
    auto indices = std::make_unique<size_type[]>(_nVertices);
    for (size_type iVertex = 0; iVertex < _nVertices; ++iVertex) {
      auto currentVertex = (*this)[iVertex];
      bool found = false;
      for (size_type jResultVertex = 0; jResultVertex < nResultVertices;
           ++jResultVertex) {
        if (currentVertex == resultVertexList[jResultVertex]) {
          indices[iVertex] = jResultVertex;
          found = true;
          break;
        }
      }
      if (!found) {
        resultVertexList[nResultVertices] = currentVertex;
        indices[iVertex] = nResultVertices;
        ++nResultVertices;
      }
    }
    return {
        resultVertexList._copyFirstNVertices(nResultVertices),
        std::move(indices),
    };
  }

private:
  size_type _nVertices;
  map_channel_size_type _nMaps;
  std::unique_ptr<Point3[]> _vertexChannel;
  std::unique_ptr<Point3[]> _normalChannel;
  std::unique_ptr<Point3[]> _texcoordChannel;
  std::unique_ptr<std::unique_ptr<Point3[]>[]> _mapChannels;

  vertex_list _copyFirstNVertices(size_type n_) {
    vertex_list result{n_, _nMaps};
    std::copy_n(_vertexChannel.get(), n_, result._vertexChannel.get());
    std::copy_n(_normalChannel.get(), n_, result._normalChannel.get());
    std::copy_n(_texcoordChannel.get(), n_, result._texcoordChannel.get());
    for (map_channel_size_type iMap = 0; iMap < _nMaps; ++iMap) {
      std::copy_n(_mapChannels[iMap].get(), n_,
                  result._mapChannels[iMap].get());
    }
    return result;
  }
};
} // namespace fant