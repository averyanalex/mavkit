use crate::types::{ParamOperationKind, SyncState};
use serde::{Deserialize, Serialize};
use std::collections::hash_map;
use std::collections::HashMap;

/// MAVLink parameter value type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParamType {
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Real32,
}

/// A single vehicle parameter with name, value, type, and index.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Param {
    pub name: String,
    pub value: f32,
    pub param_type: ParamType,
    pub index: u16,
}

/// In-memory store of all downloaded vehicle parameters.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ParamStore {
    pub params: HashMap<String, Param>,
    pub expected_count: u16,
}

impl ParamStore {
    /// Look up a parameter by name.
    pub fn get(&self, name: &str) -> Option<&Param> {
        self.params.get(name)
    }

    /// Check whether a parameter with the given name exists.
    pub fn contains(&self, name: &str) -> bool {
        self.params.contains_key(name)
    }

    /// Number of parameters in the store.
    pub fn len(&self) -> usize {
        self.params.len()
    }

    /// Whether the store is empty.
    pub fn is_empty(&self) -> bool {
        self.params.is_empty()
    }

    /// Iterate over all `(name, param)` pairs.
    pub fn iter(&self) -> hash_map::Iter<'_, String, Param> {
        self.params.iter()
    }

    /// Iterate over all parameter names.
    pub fn names(&self) -> hash_map::Keys<'_, String, Param> {
        self.params.keys()
    }
}

impl<'a> IntoIterator for &'a ParamStore {
    type Item = (&'a String, &'a Param);
    type IntoIter = hash_map::Iter<'a, String, Param>;

    fn into_iter(self) -> Self::IntoIter {
        self.params.iter()
    }
}

impl IntoIterator for ParamStore {
    type Item = (String, Param);
    type IntoIter = hash_map::IntoIter<String, Param>;

    fn into_iter(self) -> Self::IntoIter {
        self.params.into_iter()
    }
}

/// Result of a single parameter write, with requested and confirmed values.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamWriteResult {
    pub name: String,
    pub requested_value: f32,
    pub confirmed_value: f32,
    pub success: bool,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
/// Cached parameter-domain state plus sync and active-operation markers.
pub struct ParamState {
    pub store: Option<ParamStore>,
    pub sync: SyncState,
    pub active_op: Option<ParamOperationKind>,
}
