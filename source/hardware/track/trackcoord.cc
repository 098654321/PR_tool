#include "./trackcoord.hh"

namespace PR_tool::hardware {

    TrackCoord::TrackCoord(std::i64 r, std::i64 c, TrackDirection d, std::usize i) :
        Coord{r, c},
        dir{d}, index{i} 
    {
    }

    TrackCoord::TrackCoord() :
        TrackCoord{0, 0, TrackDirection::Vertical, 0}
    {
    }

    bool TrackCoord::operator == (const TrackCoord& other) const {
        return this->row == other.row &&
                this->col == other.col &&
                this->dir == other.dir &&
                this->index == other.index;
    }

    bool TrackCoord::operator<(const TrackCoord& other) const {
        if (row != other.row) return row < other.row;
        if (col != other.col) return col < other.col;
        if (dir != other.dir) {
            return static_cast<int>(dir) < static_cast<int>(other.dir);
        }
        return index < other.index;
    }

    auto TrackCoord::to_string() const -> std::String {
        return std::format("{}", *this);
    }

}

namespace std {

    std::size_t hash<PR_tool::hardware::TrackDirection>::operator() (const PR_tool::hardware::TrackDirection& dir) const noexcept {
        if (dir == PR_tool::hardware::TrackDirection::Horizontal) {
            return 0xCCCCCCCCCCCCCCCCULL;
        } else {
            return 0x3333333333333333ULL;
        }
    }

    std::size_t hash<PR_tool::hardware::TrackCoord>::operator() (const PR_tool::hardware::TrackCoord& coord) const noexcept {
        auto coord_hash = hash<PR_tool::hardware::Coord>();
        auto usize_hash = hash<std::size_t>{};
        auto dir_hash = hash<PR_tool::hardware::TrackDirection>{};
        return coord_hash(coord) ^
                dir_hash(coord.dir) ^
                usize_hash(coord.index);
    }

}