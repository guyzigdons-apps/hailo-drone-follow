"""
Pluggable gallery strategies for ReID matching.

Each strategy manages a gallery of known persons and decides:
- How to match a new embedding against the gallery
- Whether/how to update gallery embeddings after a match
"""

import numpy as np
from typing import Optional, Tuple


class GalleryStrategy:
    """Base class for gallery update strategies."""

    def __init__(self):
        self._names = []  # person names

    @property
    def names(self):
        return list(self._names)

    @property
    def size(self):
        return len(self._names)

    def add_person(self, name: str, embedding: np.ndarray):
        """Register a new person in the gallery."""
        raise NotImplementedError

    def match(self, embedding: np.ndarray, threshold: float) -> Tuple[Optional[str], float]:
        """
        Match embedding against gallery.
        Returns (person_name, similarity) if match found, else (None, best_similarity).
        """
        raise NotImplementedError

    def update(self, person_name: str, embedding: np.ndarray, frame_count: int):
        """Called after a successful match. Strategy decides whether to update."""
        pass  # default: do nothing


class FirstOnlyStrategy(GalleryStrategy):
    """Keep only the first-seen embedding per person. Never update."""

    def __init__(self):
        super().__init__()
        self._embeddings = []  # one per person

    def add_person(self, name: str, embedding: np.ndarray):
        self._names.append(name)
        self._embeddings.append(embedding.copy())

    def match(self, embedding: np.ndarray, threshold: float) -> Tuple[Optional[str], float]:
        if not self._embeddings:
            return None, 0.0
        gallery_matrix = np.stack(self._embeddings)
        similarities = gallery_matrix @ embedding
        best_idx = int(np.argmax(similarities))
        best_sim = float(similarities[best_idx])
        if best_sim >= threshold:
            return self._names[best_idx], best_sim
        return None, best_sim


class RunningAverageStrategy(GalleryStrategy):
    """Gallery embedding = running average of all matched embeddings, re-normalized."""

    def __init__(self):
        super().__init__()
        self._embeddings = []  # current average embedding per person
        self._counts = []      # number of embeddings averaged

    def add_person(self, name: str, embedding: np.ndarray):
        self._names.append(name)
        self._embeddings.append(embedding.copy())
        self._counts.append(1)

    def match(self, embedding: np.ndarray, threshold: float) -> Tuple[Optional[str], float]:
        if not self._embeddings:
            return None, 0.0
        gallery_matrix = np.stack(self._embeddings)
        similarities = gallery_matrix @ embedding
        best_idx = int(np.argmax(similarities))
        best_sim = float(similarities[best_idx])
        if best_sim >= threshold:
            return self._names[best_idx], best_sim
        return None, best_sim

    def update(self, person_name: str, embedding: np.ndarray, frame_count: int):
        idx = self._names.index(person_name)
        n = self._counts[idx]
        # Incremental average
        avg = (self._embeddings[idx] * n + embedding) / (n + 1)
        # Re-normalize to unit length
        norm = np.linalg.norm(avg)
        if norm > 0:
            avg = avg / norm
        self._embeddings[idx] = avg
        self._counts[idx] = n + 1


class UpdateEveryNStrategy(GalleryStrategy):
    """Replace gallery embedding every N matched frames."""

    def __init__(self, n: int = 10):
        super().__init__()
        self._embeddings = []
        self._match_counts = []
        self._n = n

    def add_person(self, name: str, embedding: np.ndarray):
        self._names.append(name)
        self._embeddings.append(embedding.copy())
        self._match_counts.append(0)

    def match(self, embedding: np.ndarray, threshold: float) -> Tuple[Optional[str], float]:
        if not self._embeddings:
            return None, 0.0
        gallery_matrix = np.stack(self._embeddings)
        similarities = gallery_matrix @ embedding
        best_idx = int(np.argmax(similarities))
        best_sim = float(similarities[best_idx])
        if best_sim >= threshold:
            return self._names[best_idx], best_sim
        return None, best_sim

    def update(self, person_name: str, embedding: np.ndarray, frame_count: int):
        idx = self._names.index(person_name)
        self._match_counts[idx] += 1
        if self._match_counts[idx] % self._n == 0:
            self._embeddings[idx] = embedding.copy()


class MultiEmbeddingStrategy(GalleryStrategy):
    """
    Store up to K embeddings per person.
    Match = max similarity across all stored embeddings for a person.
    """

    def __init__(self, max_k: int = 10):
        super().__init__()
        self._person_embeddings = {}  # name -> list of embeddings
        self._max_k = max_k

    def add_person(self, name: str, embedding: np.ndarray):
        self._names.append(name)
        self._person_embeddings[name] = [embedding.copy()]

    def match(self, embedding: np.ndarray, threshold: float) -> Tuple[Optional[str], float]:
        if not self._names:
            return None, 0.0
        best_name = None
        best_sim = -1.0
        for name in self._names:
            embs = np.stack(self._person_embeddings[name])
            sims = embs @ embedding
            max_sim = float(np.max(sims))
            if max_sim > best_sim:
                best_sim = max_sim
                best_name = name
        if best_sim >= threshold:
            return best_name, best_sim
        return None, best_sim

    def update(self, person_name: str, embedding: np.ndarray, frame_count: int):
        embs = self._person_embeddings[person_name]
        if len(embs) < self._max_k:
            embs.append(embedding.copy())
        else:
            # Replace the oldest embedding
            embs.pop(0)
            embs.append(embedding.copy())


# Strategy factory
STRATEGIES = {
    "first_only": FirstOnlyStrategy,
    "running_average": RunningAverageStrategy,
    "update_every_n": UpdateEveryNStrategy,
    "multi_embedding": MultiEmbeddingStrategy,
}


def create_strategy(name: str, **kwargs) -> GalleryStrategy:
    """Create a gallery strategy by name."""
    if name not in STRATEGIES:
        raise ValueError(f"Unknown strategy: {name}. Available: {list(STRATEGIES.keys())}")
    return STRATEGIES[name](**kwargs)
