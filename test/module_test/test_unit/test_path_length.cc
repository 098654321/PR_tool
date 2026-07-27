#include <algo/router/common/maze/path_length.hh>
#include <hardware/track/track.hh>
#include <std/collection.hh>

#include "./utilty.hh"

using namespace PR_tool::algo;
using namespace PR_tool::hardware;

// COB adjacency follows track_pos_to_cobs in path_length.cc.
// - t1,t2,t3 share COB (5,10); t3,t4,t5 share COB (5,11) (pivot t3 must appear in both stars).
// - t6 is an extra segment after t5 on the same tile (H then V at 5,12), sharing COB (5,12) with t5.
// Compressed length: 2 + (2-1) + (2-1) = 4.  A five-track path ending at t5 would give 3; the sixth
// track adds another two-track star (+1 net).  (Without pivot overlap fix, the middle star was wrong.)
static void test_path_length_back_to_back_cob_stars() {
    std::Vector<Track> tracks {};
    tracks.reserve(6);
    tracks.emplace_back(5, 10, TrackDirection::Vertical, 0);
    tracks.emplace_back(5, 10, TrackDirection::Horizontal, 0);
    tracks.emplace_back(5, 11, TrackDirection::Horizontal, 0);
    tracks.emplace_back(6, 11, TrackDirection::Vertical, 0);
    tracks.emplace_back(5, 12, TrackDirection::Horizontal, 0);
    tracks.emplace_back(5, 12, TrackDirection::Vertical, 0);

    std::Vector<Track*> path {};
    for (auto& t : tracks) {
        path.emplace_back(&t);
    }

    ASSERT_EQ(path_length(path, false), 4u);
    ASSERT_EQ(path_length(path, true), path.size());
}

// Same geometry as above but stops at t5: two overlapping three-track stars only → length 3.
static void test_path_length_back_to_back_cob_stars_five_tracks() {
    std::Vector<Track> tracks {};
    tracks.emplace_back(5, 10, TrackDirection::Vertical, 0);
    tracks.emplace_back(5, 10, TrackDirection::Horizontal, 0);
    tracks.emplace_back(5, 11, TrackDirection::Horizontal, 0);
    tracks.emplace_back(6, 11, TrackDirection::Vertical, 0);
    tracks.emplace_back(5, 12, TrackDirection::Horizontal, 0);

    std::Vector<Track*> path {};
    for (auto& t : tracks) {
        path.emplace_back(&t);
    }

    ASSERT_EQ(path_length(path, false), 3u);
}

// One COB star with three tracks: contribution 2.
static void test_path_length_single_cob_star_three_tracks() {
    std::Vector<Track> tracks {};
    tracks.emplace_back(5, 10, TrackDirection::Vertical, 0);
    tracks.emplace_back(5, 10, TrackDirection::Horizontal, 0);
    tracks.emplace_back(5, 11, TrackDirection::Horizontal, 0);

    std::Vector<Track*> path {};
    for (auto& t : tracks) {
        path.emplace_back(&t);
    }

    ASSERT_EQ(path_length(path, false), 2u);
}

void test_path_length_main() {
    test_path_length_back_to_back_cob_stars();
    test_path_length_back_to_back_cob_stars_five_tracks();
    test_path_length_single_cob_star_three_tracks();
}
